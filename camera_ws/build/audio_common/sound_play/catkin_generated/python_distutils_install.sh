#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/src/audio_common/sound_play"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/install/lib/python3/dist-packages:/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/build" \
    "/bin/python3" \
    "/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/src/audio_common/sound_play/setup.py" \
    egg_info --egg-base /home/jianrenw/Documents/skilled/cobot_magic/camera_ws/build/audio_common/sound_play \
    build --build-base "/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/build/audio_common/sound_play" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/install" --install-scripts="/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/install/bin"
