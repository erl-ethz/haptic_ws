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

echo_and_run cd "/home/emanuele/catkin_ws/src/CrazyS/rqt_rotors"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/emanuele/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/emanuele/catkin_ws/install/lib/python2.7/dist-packages:/home/emanuele/catkin_ws/build/rqt_rotors/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/emanuele/catkin_ws/build/rqt_rotors" \
    "/usr/bin/python2" \
    "/home/emanuele/catkin_ws/src/CrazyS/rqt_rotors/setup.py" \
     \
    build --build-base "/home/emanuele/catkin_ws/build/rqt_rotors" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/emanuele/catkin_ws/install" --install-scripts="/home/emanuele/catkin_ws/install/bin"
