# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
"""
Backbone modules.
"""
from collections import OrderedDict
import yaml
from omegaconf import OmegaConf, DictConfig
from hydra.utils import instantiate
import torch
import torch.nn.functional as F
import torchvision
from torch import nn
from torchvision.models._utils import IntermediateLayerGetter
from typing import Dict, List
import pickle
import os
import numpy as np
from ..util.misc import NestedTensor, is_main_process
from detr.util.misc import replace_submodules
from .position_encoding import build_position_encoding
from transformers import ASTModel, ASTConfig, ASTFeatureExtractor
import IPython
e = IPython.embed

class FrozenBatchNorm2d(torch.nn.Module):
    """
    BatchNorm2d where the batch statistics and the affine parameters are fixed.

    Copy-paste from torchvision.misc.ops with added eps before rqsrt,
    without which any other policy_models than torchvision.policy_models.resnet[18,34,50,101]
    produce nans.
    """

    def __init__(self, n):
        super(FrozenBatchNorm2d, self).__init__()
        self.register_buffer("weight", torch.ones(n))
        self.register_buffer("bias", torch.zeros(n))
        self.register_buffer("running_mean", torch.zeros(n))
        self.register_buffer("running_var", torch.ones(n))

    def _load_from_state_dict(self, state_dict, prefix, local_metadata, strict,
                              missing_keys, unexpected_keys, error_msgs):
        num_batches_tracked_key = prefix + 'num_batches_tracked'
        if num_batches_tracked_key in state_dict:
            del state_dict[num_batches_tracked_key]

        super(FrozenBatchNorm2d, self)._load_from_state_dict(
            state_dict, prefix, local_metadata, strict,
            missing_keys, unexpected_keys, error_msgs)

    def forward(self, x):
        # move reshapes to the beginning
        # to make it fuser-friendly
        w = self.weight.reshape(1, -1, 1, 1)
        b = self.bias.reshape(1, -1, 1, 1)
        rv = self.running_var.reshape(1, -1, 1, 1)
        rm = self.running_mean.reshape(1, -1, 1, 1)
        eps = 1e-5
        scale = w * (rv + eps).rsqrt()
        bias = b - rm * scale
        return x * scale + bias


class BackboneBase(nn.Module):

    def __init__(self, backbone: nn.Module, train_backbone: bool, num_channels: int, return_interm_layers: bool):
        super().__init__()
        # for name, parameter in backbone.named_parameters(): # only train later layers # TODO do we want this?
        #     if not train_backbone or 'layer2' not in name and 'layer3' not in name and 'layer4' not in name:
        #         parameter.requires_grad_(False)
        if return_interm_layers:
            return_layers = {"layer1": "0", "layer2": "1", "layer3": "2", "layer4": "3"}
        else:
            return_layers = {'layer4': "0"}
        self.body = IntermediateLayerGetter(backbone, return_layers=return_layers)
        self.num_channels = num_channels

    def forward(self, tensor):
        xs = self.body(tensor)
        return xs  # expected output: {"0": an array of shape (num_channel,)}
        # out: Dict[str, NestedTensor] = {}
        # for name, x in xs.items():
        #     m = tensor_list.mask
        #     assert m is not None
        #     mask = F.interpolate(m[None].float(), size=x.shape[-2:]).to(torch.bool)[0]
        #     out[name] = NestedTensor(x, mask)
        # return out


class Backbone(BackboneBase):
    """ResNet backbone with frozen BatchNorm."""
    def __init__(self, name: str,
                 train_backbone: bool,
                 return_interm_layers: bool,
                 dilation: bool):
        backbone = getattr(torchvision.models, name)(
            replace_stride_with_dilation=[False, False, dilation],
            pretrained=is_main_process(), norm_layer=FrozenBatchNorm2d) # pretrained # TODO do we want frozen batch_norm??

        num_channels = 512 if name in ('resnet18', 'resnet34') else 2048

        super().__init__(backbone, train_backbone, num_channels, return_interm_layers)


class Joiner(nn.Sequential):
    def __init__(self, backbone, position_embedding):
        super().__init__(backbone, position_embedding)

    def forward(self, tensor_list: NestedTensor):
        xs = self[0](tensor_list)
        out: List[NestedTensor] = []
        pos = []
        for name, x in xs.items():
            out.append(x)
            # position encoding
            pos.append(self[1](x).to(x.dtype))

        return out, pos


def build_backbone(args):
    position_embedding = build_position_encoding(args)
    train_backbone = args.lr_backbone > 0
    return_interm_layers = args.masks
    backbone = Backbone(args.backbone, train_backbone, return_interm_layers, args.dilation)
    model = Joiner(backbone, position_embedding)
    model.num_channels = backbone.num_channels
    return model


class RestNetBasicBlock(nn.Module):
    def __init__(self, in_channels, out_channels, stride):
        super(RestNetBasicBlock, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=stride, padding=1)
        self.bn2 = nn.BatchNorm2d(out_channels)

    def forward(self, x):
        output = self.conv1(x)
        output = F.relu(self.bn1(output))
        output = self.conv2(output)
        output = self.bn2(output)
        return F.relu(x + output)

def load_yaml_cfg(path):
    try:
        with open(path, 'r') as file:
            config = yaml.safe_load(file)
        return config
    except FileNotFoundError:
        print(f"Error: The file at {path} was not found.")
        return None
    except yaml.YAMLError as exc:
        print(f"Error parsing YAML file: {exc}")
        return None

def instantiate_transforms(transforms_list):
    instantiated_transforms = []
    for transform_cfg in transforms_list:
        instantiated_transforms.append(instantiate(transform_cfg))
    return instantiated_transforms

def load_audio_cfg(path_to_cfg):
    config = load_yaml_cfg(path_to_cfg)
    if config is None:
        return None
    cfg = OmegaConf.create(config)
    if 'transforms' in cfg:
        cfg.transforms = instantiate_transforms(cfg.transforms)
    config_dict = OmegaConf.to_container(cfg, resolve=True)
    return config_dict

class AudioBackbone(nn.Module):

    def __init__(self, 
                audio_cfg, ## loaded from yaml file
                ):
        super().__init__()
        if audio_cfg.name == 'ast':
            config = ASTConfig()
            config.max_length = audio_cfg.max_length
            config.num_mel_bins = audio_cfg.num_mel_bins
            self.audio_model = ASTModel(config)
            self.ast_feature_extractor = ASTFeatureExtractor(
                num_mel_bins=audio_cfg.num_mel_bins, 
                max_length=audio_cfg.max_length,
                do_normalize=False,
                # mean=audio_encoder_cfg.norm_spec.mean,
                # std=audio_encoder_cfg.norm_spec.std
            )
        else:
            raise NotImplementedError(f"Audio backbone {audio_cfg.name} is not implemented")
        
        ## TODO change the normalization in data_loader
        """ if audio_cfg.norm_spec.is_norm: # normalize to -1 and 1
            if os.path.exists(f'{audio_cfg.norm_spec.stats_dir}/spec_stats.pkl'):
                with open(f'{audio_cfg.norm_spec.stats_dir}/spec_stats.pkl', 'rb') as f:
                    stats = pickle.load(f)
                    audio_cfg.norm_spec.min = stats['mic_0']['min']
                    audio_cfg.norm_spec.max = stats['mic_0']['max']
         """
        self.feature_dim = 768 ## hardcoded for ast model
        
        # replace batchnorm by group norm
        if audio_cfg.use_group_norm:
            audio_model = replace_submodules(
                root_module=audio_model,
                predicate=lambda x: isinstance(x, nn.BatchNorm2d),
                func=lambda x: nn.GroupNorm(
                    num_groups=(x.num_features // 16) if (x.num_features % 16 == 0) else (x.num_features // 8), 
                    num_channels=x.num_features)
                )
        self.transform = nn.Identity() if audio_cfg.transforms is None else torch.nn.Sequential(*audio_cfg.transforms)
        self.audio_cfg = audio_cfg
        

    def forward(self, audio_data):
        """
        Assume audio input is B, T
        B: Batch size - The number of audio samples in a batch.
        T: total timesteps, a float per timesetp
        
        return:
        features extracted from the audio input
        """
        B, T = audio_data.shape[:2]
        #audio = audio.reshape(B, audio.shape[-1]*T)
        audio = self.transform[0](audio)
        audio = self.ast_feature_extractor(audio.cpu().numpy(), sampling_rate=16000)['input_values']
        """ if self.audio_cfg.norm_spec.is_norm: # normalize to -1 and 1
            audio = (np.array(audio) - self.audio_cfg.norm_spec.min) \
                / (self.audio_cfg.norm_spec.max - self.audio_cfg.norm_spec.min)
            audio = audio * 2 - 1 """
        audio = torch.tensor(np.array(audio)).to(self.device)
        # Apply remaining transforms
        for transform in self.transform[1:]:
            audio = transform(audio)

        if self.audio_cfg.mask_robot:
            audio[:, :, :8] = 0
        raw_feature = self.audio_model(audio, output_hidden_states=True).last_hidden_state
        feature = self.aggregate_feature(raw_feature,
                                             model_name=self.audio_encoder_cfg.model_name,
                                             agg_mode=self.audio_encoder_cfg.feature_aggregation)
        return feature.reshape(B, -1)

    def aggregate_feature(self, feature, model_name, agg_mode):
        if model_name.startswith('ast'):
            if agg_mode == 'all_tokens':
                return feature
            elif agg_mode is None: # use the CLS token
                return feature[:, 0, :]
        else:
            raise NotImplementedError(f"model {model_name} is not implemented")