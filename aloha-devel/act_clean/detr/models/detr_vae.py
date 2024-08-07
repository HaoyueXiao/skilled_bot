# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
"""
DETR model and criterion classes.
"""
import torch
from torch import nn
from torch.autograd import Variable
from .backbone import build_backbone, load_audio_cfg, AudioBackbone
from .transformer import build_transformer, TransformerEncoder, TransformerEncoderLayer

import numpy as np
from collections import OrderedDict
import sys
sys.path.append("./")
from robomimic.models.base_nets import ResNet18Conv, SpatialSoftmax
from robomimic.algo.diffusion_policy import replace_bn_with_gn, ConditionalUnet1D
import os
import pickle
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.schedulers.scheduling_ddim import DDIMScheduler
from diffusers.training_utils import EMAModel
import timm
from transformers import ASTModel, ASTConfig, ASTFeatureExtractor
import IPython
from detr.util.misc import replace_submodules
e = IPython.embed


def reparametrize(mu, logvar):
    std = logvar.div(2).exp()
    eps = Variable(std.data.new(std.size()).normal_())
    return mu + std * eps


def get_sinusoid_encoding_table(n_position, d_hid):
    def get_position_angle_vec(position):
        return [position / np.power(10000, 2 * (hid_j // 2) / d_hid) for hid_j in range(d_hid)]

    sinusoid_table = np.array([get_position_angle_vec(pos_i) for pos_i in range(n_position)])
    sinusoid_table[:, 0::2] = np.sin(sinusoid_table[:, 0::2])  # dim 2i
    sinusoid_table[:, 1::2] = np.cos(sinusoid_table[:, 1::2])  # dim 2i+1

    return torch.FloatTensor(sinusoid_table).unsqueeze(0)


class DETRVAE(nn.Module):
    """ This is the DETR module that performs object detection """
    def __init__(self, backbones,
                depth_backbones,
                transformer, 
                encoder, 
                state_dim, 
                num_queries, 
                camera_names, 
                kl_weight, 
                audio_models): ## audio_model: a list of audio encoders
        """ Initializes the model.
        Parameters:
            backbones: torch module of the backbone to be used. See backbone.py
            transformer: torch module of the transformer architecture. See transformer.py
            state_dim: robot state dimension of the environment
            num_queries: number of object queries, ie detection slot. This is the maximal number of objects
                         DETR can detect in a single image. For COCO, we recommend 100 queries.
            aux_loss: True if auxiliary decoding losses (loss at each decoder layer) are to be used.
        """
        super().__init__()
        self.num_queries = num_queries
        self.camera_names = camera_names
        self.transformer = transformer
        self.encoder = encoder
        self.audio_models = audio_models
        hidden_dim = transformer.d_model
        self.action_head = nn.Linear(hidden_dim, state_dim)
        self.query_embed = nn.Embedding(num_queries, hidden_dim)
        self.kl_weight = kl_weight

        if backbones is not None:
            # print("backbones[0]", backbones[0])
            self.depth_backbones = None
            self.input_proj = nn.Conv2d(backbones[0].num_channels, hidden_dim, kernel_size=1)
            self.backbones = nn.ModuleList(backbones)
            self.input_proj_robot_state = nn.Linear(state_dim, hidden_dim)
        
        else:
            # input_dim = 14 + 7 # robot_state + env_state
            self.input_proj_robot_state = nn.Linear(state_dim, hidden_dim)
            self.pos = torch.nn.Embedding(2, hidden_dim)
            self.backbones = None

        if audio_models:
            self.audio_input_proj = nn.Linear(audio_models.feature_dim, hidden_dim)

        self.audio_embeddings = []
        for _ in audio_models:
            self.audio_embeddings.append(nn.Embedding(1, hidden_dim))

        # encoder extra parameters
        self.latent_dim = 32  # final size of latent z # TODO tune
        self.cls_embed = nn.Embedding(1, hidden_dim)  # extra cls token embedding

        # decoder extra parameters
        self.latent_out_proj = nn.Linear(self.latent_dim, hidden_dim)  # project latent sample to embedding

        self.latent_pos = nn.Embedding(1, hidden_dim)
        self.robot_state_pos = nn.Embedding(1, hidden_dim)

        if kl_weight != 0:
            self.encoder_action_proj = nn.Linear(state_dim, hidden_dim)  # project action to embedding
            self.encoder_joint_proj = nn.Linear(state_dim, hidden_dim)  # project qpos to embedding
            self.latent_proj = nn.Linear(hidden_dim, self.latent_dim*2)  # project hidden state to latent std, var
            self.register_buffer('pos_table', get_sinusoid_encoding_table(1+1+num_queries, hidden_dim))  # [CLS], qpos, a_seq

    def forward(self, image, depth_image, audio_data, robot_state, actions=None, action_is_pad=None):
        """
        qpos: batch, qpos_dim
        image: batch, num_cam, channel, height, width
        env_state: None                                   没有值
        actions: batch, seq, action_dim                    
        """

        # print("forward: ", qpos.shape, image.shape, env_state, actions.shape, action_is_pad.shape)

        is_training = actions is not None  # train or val
        bs, _ = robot_state.shape

        # Obtain latent z from action sequence
        if is_training and self.kl_weight != 0:  # hidden_dim输入参数是512
            action_embed = self.encoder_action_proj(actions)  # (bs, seq, hidden_dim)
            robot_state_embed = self.encoder_joint_proj(robot_state)  # (bs, hidden_dim)
            robot_state_embed = torch.unsqueeze(robot_state_embed, axis=1)  # (bs, 1, hidden_dim)
            cls_embed = self.cls_embed.weight  # (1, hidden_dim)
            cls_embed = torch.unsqueeze(cls_embed, axis=0).repeat(bs, 1, 1)  # (bs, 1, hidden_dim)
            encoder_input = torch.cat([cls_embed, robot_state_embed, action_embed], axis=1)  # (bs, seq+2, hidden_dim)
            encoder_input = encoder_input.permute(1, 0, 2)  # (seq+2, bs, hidden_dim)
            cls_joint_is_pad = torch.full((bs, 2), False).to(robot_state.device)  # False: not a padding
            is_pad = torch.cat([cls_joint_is_pad, action_is_pad], axis=1)  # (bs, seq+2)

            # obtain position embedding  合并位置编码
            pos_embed = self.pos_table.clone().detach()
            pos_embed = pos_embed.permute(1, 0, 2)  # (seq+1, 1, hidden_dim)
            encoder_output = self.encoder(encoder_input, pos=pos_embed, src_key_padding_mask=is_pad)
            encoder_output = encoder_output[0]  # take cls output only
            
            # 线性层  hidden_dim扩大到64
            latent_info = self.latent_proj(encoder_output)
            mu = latent_info[:, :self.latent_dim]
            logvar = latent_info[:, self.latent_dim:]
            latent_sample = reparametrize(mu, logvar)
            latent_input = self.latent_out_proj(latent_sample)
        else:
            mu = logvar = None
            latent_sample = torch.zeros([bs, self.latent_dim], dtype=torch.float32).to(robot_state.device)
            latent_input = self.latent_out_proj(latent_sample)
        
        
        # Image observation features and position embeddings
        all_cam_features = []
        all_cam_pos = []
        for cam_id, cam_name in enumerate(self.camera_names):
            # features, pos = self.backbones[0](image[:, cam_id])  # HARDCODED
            features, src_pos = self.backbones[cam_id](image[:, cam_id]) # HARDCODED
            # image_test = image[:, cam_id][:, 0].unsqueeze(dim=1)
            # print("depth_encoder:", self.depth_encoder(image_test))
            features = features[0]  # take the last layer feature
            src_pos = src_pos[0]
            all_cam_features.append(self.input_proj(features))
            all_cam_pos.append(src_pos)
        
        # audio feature processing:
        all_audio_features = []
        if audio_data is not None:
            # suppose audio_data is in shape (B, N, D), where N is the number of audio devices, 2 in our case
            for audio_id, audio_name in enumerate(audio_data):
                audio_features = self.audio_models[audio_id](audio_data[:, audio_id]) # (B, 768)
                audio_features = self.audio_input_proj(audio_features).unsqueeze(1) # project from 768 to 512 and unsqueeze
                all_audio_features.append(audio_features) # (B, 1, 512)
            audio_src = torch.cat(all_audio_features, axis = 1).permute(1, 0, 2) # (2, B, 512)
            audio_pos = torch.cat([audio_embed.weight for audio_embed in self.audio_embeddings], axis = 0) # (2, 512)
        else:
            audio_src = None
            audio_pos = None
        # proprioception features
        robot_state_input = self.input_proj_robot_state(robot_state)
        robot_state_input = torch.unsqueeze(robot_state_input, axis=0) #(1, B, 512)
        latent_input = torch.unsqueeze(latent_input, axis=0) #(1, B, 512)
        
        
        # fold camera dimension into width dimension
        src = torch.cat(all_cam_features, axis=3)
        src_pos = torch.cat(all_cam_pos, axis=3)
        hs = self.transformer(self.query_embed.weight,
                              src, src_pos, None,
                              robot_state_input, self.robot_state_pos.weight,
                              latent_input, self.latent_pos.weight, audio_src, audio_pos)[0]
        a_hat = self.action_head(hs)
        return a_hat, [mu, logvar]


def build_visual_encoder(args):
    
    d_model = args.hidden_dim  # 256
    dropout = args.dropout     # 0.1
    nhead = args.nheads        # 8
    dim_feedforward = args.dim_feedforward  # 2048
    num_encoder_layers = args.enc_layers  # 4 # TODO shared with VAE decoder
    normalize_before = args.pre_norm  # False
    activation = "relu"

    encoder_layer = TransformerEncoderLayer(d_model, nhead, dim_feedforward,
                                            dropout, activation, normalize_before)
    
    encoder_norm = nn.LayerNorm(d_model) if normalize_before else None
    
    encoder = TransformerEncoder(encoder_layer, num_encoder_layers, encoder_norm)

    return encoder
    

def build(args):
    if args.use_robot_base:
        state_dim = 16  # TODO hardcode
    else:
        state_dim = 14

    # From state
    # backbone = None # from state for now, no need for conv nets
    # From image
    backbones = []   # 空的网络list
    depth_backbones = None

    # backbone = build_backbone(args)  # 位置编码和主干网络组合成特征提取器
    # backbones.append(backbone)
    # if args.use_depth_image:
    #     depth_backbones.append(DepthNet())

    for _ in args.camera_names:
        backbone = build_backbone(args)
        backbones.append(backbone)

    transformer = build_transformer(args)  # 构建trans层

    encoder = None
    if args.kl_weight != 0:
        encoder = build_visual_encoder(args)          # 构建编码成和解码层
    
    audio_models = []
    if args.audio_devices:
        cfg = load_audio_cfg(args.audio_cfg)
        for _ in args.audio_devices:
            audio_models.append(AudioBackbone(cfg))

    model = DETRVAE(
        backbones,
        depth_backbones,
        transformer,
        encoder,
        state_dim=state_dim,
        num_queries=args.chunk_size,
        camera_names=args.camera_names,
        kl_weight=args.kl_weight,
        audio_models=audio_models
        )

    n_parameters = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print("number of parameters: %.2fM" % (n_parameters/1e6,))

    return model
