all:
	$(MAKE) -C drumrobot_server

clean:
	$(MAKE) -C drumrobot_server clean

# 함께 띄울 것들 (끄려면: make run CLIENT=0 / make run PLOTTER=0)
CLIENT  ?= 1
PLOTTER ?= 1

# 음악 서버 통로, 작업 폴더, 쿠키
run:
	$(MAKE) -C drumrobot_server
	@sudo -v
	@CLIENT=$(CLIENT) PLOTTER=$(PLOTTER) ./scripts/run_side_apps.sh &
	sudo PULSE_SERVER=unix:/run/user/$(shell id -u)/pulse/native \
	     XDG_RUNTIME_DIR=/run/user/$(shell id -u) \
	     PULSE_COOKIE=$(HOME)/.config/pulse/cookie \
	     ./drumrobot_server/bin/main.out