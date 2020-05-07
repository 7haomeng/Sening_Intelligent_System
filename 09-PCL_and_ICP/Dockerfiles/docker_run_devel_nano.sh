xhost +local:docker
nvidia-docker run --name lab5 -it --net=host --privileged -v /dev:/dev \
	--runtime nvidia \
	-e DISPLAY=$DISPLAY \
	--env="QT_X11_NO_MITSHM=1" \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-env="XAUTHORITY=$XAUTH" \
	--volume="$XAUTH:$XAUTH" \
	--runtime=nvidia \
	--device=/dev/dri:/dev/dri \
	--device=/dev/nvhost-ctrl \
	--device=/dev/nvhost-ctrl-gpu \
	--device=/dev/nvhost-prof-gpu \
	--device=/dev/nvmap \
	--device=/dev/nvhost-gpu \
	--device=/dev/nvhost-as-gpu \
	--device=/dev/ttyUSB0 \
	-v /dev/bus/usb:/dev/bus/usb \
	lab5 bash
