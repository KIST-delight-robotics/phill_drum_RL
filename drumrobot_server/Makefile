# ===== 컴파일러 설정 =====
CC = g++

# ===== 컴파일 옵션 =====
CFLAGS = -Wall -O2 -g -std=c++17 -fPIC

# 헤더 의존성 자동 생성. 없으면 헤더만 고쳤을 때 .o 가 낡은 채로 남아
# "undefined reference" 같은 엉뚱한 링크 에러가 난다.
DEPFLAGS = -MMD -MP

# ===== ONNX Runtime (CPU) =====
# lib/onnxruntime/fetch.sh 로 받는다. 커밋하지 않는다.
# $ORIGIN 상대 rpath 로 링크하므로 로봇 머신에 시스템 설치가 필요 없다.
ORT_DIR = ./lib/onnxruntime

# ===== 헤더 파일 경로 =====
INCLUDE = -I./include -I./lib -I./lib/dynamixel_sdk/include -I$(ORT_DIR)/include

# ===== 링크할 라이브러리 =====
# (멀티스레드) (수학) (동적 로딩)
LDFLAGS = -lpthread -lm -ldl \
          -L$(ORT_DIR)/lib -lonnxruntime \
          -Wl,-rpath,'$$ORIGIN/../lib/onnxruntime/lib'

# ===== 경로 변수 =====
SRCDIR = ./src
OBJDIR = ./obj
BINDIR = ./bin
EXECUTABLE = main.out
SCORE = BasicFillin
TOL = 1e-4
TIP =

# ===== 소스/오브젝트 파일 자동 탐색 =====
SOURCES := $(shell find $(SRCDIR) -name '*.cpp')
OBJFILES := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))

DXL_SOURCES := $(shell find ./lib/dynamixel_sdk/src -maxdepth 1 -name '*.cpp')
DXL_OBJECTS := $(patsubst ./lib/dynamixel_sdk/src/%.cpp, $(OBJDIR)/dxl/%.o, $(DXL_SOURCES))

DEPFILES := $(OBJFILES:.o=.d) $(DXL_OBJECTS:.o=.d)

# ===== 기본 빌드 타깃 =====
all: directories $(BINDIR)/$(EXECUTABLE)

# ===== 폴더 생성 =====
directories:
	mkdir -p $(BINDIR)
	mkdir -p $(OBJDIR)/common $(OBJDIR)/hardware $(OBJDIR)/kinematics $(OBJDIR)/tcp $(OBJDIR)/realtime $(OBJDIR)/trajectory $(OBJDIR)/util $(OBJDIR)/policy
	mkdir -p $(OBJDIR)/dxl

# ===== 링크: .o 파일들 → 실행파일 =====
$(BINDIR)/$(EXECUTABLE): $(OBJFILES) $(DXL_OBJECTS)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) $(LDFLAGS)

# ===== 컴파일: src/*.cpp → .o =====
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) $(DEPFLAGS) -c $< -o $@ $(INCLUDE)

# ===== 컴파일: Dynamixel SDK *.cpp → .o =====
$(OBJDIR)/dxl/%.o: ./lib/dynamixel_sdk/src/%.cpp
	$(CC) $(CFLAGS) $(DEPFLAGS) -c $< -o $@ $(INCLUDE)

# 생성된 의존성 파일 포함 (없으면 무시)
-include $(DEPFILES)

# ===== 실행 =====
# config/log 경로가 모두 'drumrobot_server/...' 상대경로라, 그 경로가 보이는 위치에서 실행한다.
# 1) 프로젝트 루트(Phil-drum-robot)에서 바로 보이면 그대로 실행
# 2) 안 보이면 상위 디렉터리로 올라가 실행 (drumrobot_server 안에서 make run 한 경우)
run: all
	@if [ -f drumrobot_server/config/motors.json ]; then \
		sudo ./drumrobot_server/bin/$(EXECUTABLE); \
	elif [ -f ../drumrobot_server/config/motors.json ]; then \
		cd .. && sudo ./drumrobot_server/bin/$(EXECUTABLE); \
	else \
		echo "[run] drumrobot_server/config/motors.json 를 찾을 수 없습니다"; exit 1; \
	fi

# ===== ONNX Runtime 확인 =====
# vendoring 이 제대로 됐는지, export 한 정책을 C++ 에서 열 수 있는지 확인한다.
ort-check: directories
	$(CC) $(CFLAGS) tools/ort_check.cpp -o $(BINDIR)/ort_check $(INCLUDE) $(LDFLAGS)
	@echo "--- 실행 ---"
	@cd .. && ./drumrobot_server/bin/ort_check drumrobot_server/data/policy/policy.onnx

# ===== policy_score / policy_config 확인 =====
score-check: directories $(OBJDIR)/policy/policy_config.o $(OBJDIR)/policy/policy_score.o $(OBJDIR)/policy/midi_score.o
	$(CC) $(CFLAGS) tools/policy_score_check.cpp $(OBJDIR)/policy/policy_config.o \
	    $(OBJDIR)/policy/policy_score.o $(OBJDIR)/policy/midi_score.o -o $(BINDIR)/score_check $(INCLUDE) $(LDFLAGS)
	@echo "--- 실행 ---"
	@cd .. && ./drumrobot_server/bin/score_check \
	    drumrobot_server/data/policy/obs_constants.json $(SCORE)

# ===== obs_builder 확인 =====
OBS_OBJS = $(OBJDIR)/policy/policy_config.o $(OBJDIR)/policy/policy_score.o \
           $(OBJDIR)/policy/midi_score.o $(OBJDIR)/policy/obs_builder.o \
           $(OBJDIR)/kinematics/kinematics_solver.o
obs-check: directories $(OBS_OBJS)
	$(CC) $(CFLAGS) tools/obs_check.cpp $(OBS_OBJS) -o $(BINDIR)/obs_check $(INCLUDE) $(LDFLAGS)
	@echo "--- 실행 ---"
	@cd .. && ./drumrobot_server/bin/obs_check \
	    drumrobot_server/data/policy/obs_constants.json $(SCORE)

# ===== 골든 테스트 =====
golden-check: directories $(OBS_OBJS)
	$(CC) $(CFLAGS) tools/golden_check.cpp $(OBS_OBJS) -o $(BINDIR)/golden_check $(INCLUDE) $(LDFLAGS)
	@echo "--- 실행 ---"
	@cd .. && ./drumrobot_server/bin/golden_check \
	    drumrobot_server/data/policy/obs_constants.json \
	    drumrobot_server/data/policy/golden.json $(TOL) $(TIP)

# ===== 청소 =====
clean:
	rm -rf $(OBJDIR) $(BINDIR)