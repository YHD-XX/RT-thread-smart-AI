STAGE7D_CXXFLAGS = -std=gnu++17 -Wno-error=c++17-extensions -Wno-error=c++14-extensions -Wno-error=reorder -Wno-error=narrowing -Wno-error=sign-compare -Wno-error=return-type  -Wno-error=multichar -Wno-multichar -Wno-error=unused-value -mcmodel=medany -mno-relax
CXX_STD_FLAGS = -std=gnu++17 -Wno-error=c++17-extensions -Wno-error=c++14-extensions -Wno-error=reorder -Wno-error=narrowing -Wno-error=sign-compare -Wno-error=return-type
include $(MPP_SRC_DIR)/userapps/sample/mpp.mk
include $(MPP_SRC_DIR)/userapps/sample/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
BIN = $(MPP_SRC_DIR)/userapps/sample/elf/$(CURRECT_DIR_NAME).elf
ANALYZER_BIN = $(MPP_SRC_DIR)/userapps/sample/elf/sample_motion_analyzer.elf

LIBPATH = $(MPP_LIB_PATH)
LIBS = $(MPP_LIBS)

# DataFIFO: big-core VENC stream -> little-core RTSP server
DATAFIFO_DIR = $(MPP_SRC_DIR)/../../common/cdk/user/component/datafifo
DATAFIFO_INC = $(DATAFIFO_DIR)/include
DATAFIFO_LIB_PATH = $(DATAFIFO_DIR)/host/lib

JPEG_ROOT = $(MPP_SRC_DIR)/../utils/lib/hhb-prebuilt-decode
JPEG_INC = $(JPEG_ROOT)/include
JPEG_LIB_PATH = $(JPEG_ROOT)/lib

NNCASE_ROOT = $(MPP_SRC_DIR)/../nncase/riscv64/nncase
NNCASE_INC = $(NNCASE_ROOT)/include
GSL_INC = /home/ubuntu/k230_sdk/src/big/nncase/riscv64
NNCASE_LIB_PATH = $(NNCASE_ROOT)/lib
OPENCV_ROOT = $(MPP_SRC_DIR)/../utils/lib/opencv
OPENCV_INC = $(OPENCV_ROOT)/include/opencv4
OPENCV_LIB_PATH = $(OPENCV_ROOT)/lib
OPENCV_3RDPARTY_LIB_PATH = $(OPENCV_ROOT)/lib/opencv4/3rdparty
MPP_INC = $(MPP_SRC_DIR)/include
MPP_COMM_INC = $(MPP_SRC_DIR)/include/comm
MPP_USERAPPS_INC = $(MPP_SRC_DIR)/userapps/api

# OpenCV used by the independent motion analyzer

OPENCV_LIBS = -latomic \
              -lopencv_video \
              -lopencv_calib3d \
              -lopencv_features2d \
              -lopencv_flann \
              -lopencv_imgproc \
              -lopencv_core \
              -lcsi_cv \
              -lzlib

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR) \
               -I$(DATAFIFO_INC) \
               -I$(JPEG_INC) \
               -I$(NNCASE_INC) \
               -I$(GSL_INC) \
               -I$(OPENCV_ROOT)/include/opencv4

LOCAL_CPPFLAGS = $(LOCAL_CFLAGS) \
                 -std=gnu++17 \
                 -fexceptions \
                 -frtti

# All C files belong to the main process.
SRCS = $(wildcard $(LOCAL_SRC_DIR)/*.c)
OBJS = $(patsubst %.c,%.o,$(SRCS))

# motion_analyzer.cpp and motion_lk.cpp belong only to analyzer.
ALL_CPP_SRCS = $(wildcard $(LOCAL_SRC_DIR)/*.cpp)

ANALYZER_CPP_SRCS = \
    $(LOCAL_SRC_DIR)/motion_analyzer.cpp \
    $(LOCAL_SRC_DIR)/motion_lk.cpp

CPP_SRCS = $(filter-out $(ANALYZER_CPP_SRCS),$(ALL_CPP_SRCS))
CPP_OBJS = $(patsubst %.cpp,%.o,$(CPP_SRCS))

ANALYZER_CPP_OBJS = \
    $(LOCAL_SRC_DIR)/motion_analyzer.o \
    $(LOCAL_SRC_DIR)/motion_lk.o

IPC_OBJ = $(LOCAL_SRC_DIR)/motion_ipc.o
LIGHT_OBJ = $(LOCAL_SRC_DIR)/motion_light.o
VISION_OBJS = $(LOCAL_SRC_DIR)/motion_vision.o \
              $(LOCAL_SRC_DIR)/motion_ai_base.o \
              $(LOCAL_SRC_DIR)/motion_ai_utils.o \
              $(LOCAL_SRC_DIR)/motion_ob_det.o

# Stage7D: sample_venc.elf must not link vision/nncase objects.
# Vision objects are linked only into sample_motion_analyzer.elf.
MAIN_VISION_EXCLUDE = $(VISION_OBJS) \
                      $(LOCAL_SRC_DIR)/motion_ob_det.o \
                      $(LOCAL_SRC_DIR)/motion_vision.o \
                      $(LOCAL_SRC_DIR)/motion_ai_base.o \
                      $(LOCAL_SRC_DIR)/motion_ai_utils.o

OBJS := $(filter-out $(MAIN_VISION_EXCLUDE),$(OBJS))
OBJ := $(filter-out $(MAIN_VISION_EXCLUDE),$(OBJ))
C_OBJS := $(filter-out $(MAIN_VISION_EXCLUDE),$(C_OBJS))
CPP_OBJS := $(filter-out $(MAIN_VISION_EXCLUDE),$(CPP_OBJS))
CXX_OBJS := $(filter-out $(MAIN_VISION_EXCLUDE),$(CXX_OBJS))
LOCAL_OBJS := $(filter-out $(MAIN_VISION_EXCLUDE),$(LOCAL_OBJS))
SRCS_OBJS := $(filter-out $(MAIN_VISION_EXCLUDE),$(SRCS_OBJS))

all: $(BIN) $(ANALYZER_BIN)
	@-rm -f $(OBJS) $(CPP_OBJS) $(ANALYZER_CPP_OBJS)
	echo "${PWD}/Makefile all"

$(OBJS): %.o : %.c
	@echo CC $@
	@$(CC) $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) -c $< -o $@

$(CPP_OBJS) $(ANALYZER_CPP_OBJS): %.o : %.cpp
	@echo CXX $@
	@$(CPP) $(STAGE7D_CXXFLAGS) $(CXX_STD_FLAGS) $(CC_CFLAGS) $(LOCAL_CPPFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) -c $< -o $@

# Main process: capture, VENC, RTSP, OSD, snapshot and IPC producer.
$(BIN): $(OBJS) $(CPP_OBJS)
	@echo LD $@
	$(CPP) $(STAGE7D_CXXFLAGS) -o $(BIN) \
		$(LINKFLAG) \
		-Wl,--whole-archive \
		-Wl,--no-whole-archive \
		-n \
		--static \
		$(OBJS) $(CPP_OBJS) \
		-L$(LIBPATH) \
		-L$(DATAFIFO_LIB_PATH) \
		-L$(JPEG_LIB_PATH) \
		-L$(NNCASE_LIB_PATH) \
		-L$(OPENCV_LIB_PATH) \
		-L$(OPENCV_3RDPARTY_LIB_PATH) \
		-Wl,--start-group \
		$(LIBS) \
		-ldatafifo \
		-ljpeg \
		-lNncase.Runtime.Native \
		-lfunctional_k230 \
		-lnncase.rt_modules.k230 \
		$(OPENCV_LIBS) \
		-Wl,--end-group

# Analyzer process: shared-memory consumer plus OpenCV LK.
$(ANALYZER_BIN): $(ANALYZER_CPP_OBJS) $(IPC_OBJ) $(LIGHT_OBJ) $(VISION_OBJS)
	@echo LD $@
	$(CPP) $(STAGE7D_CXXFLAGS) -o $(ANALYZER_BIN) \
		$(LINKFLAG) \
		-Wl,--whole-archive \
		-Wl,--no-whole-archive \
		-n \
		--static \
		$(ANALYZER_CPP_OBJS) \
		$(IPC_OBJ) \
		$(LIGHT_OBJ) \
		$(VISION_OBJS) \
		-L$(OPENCV_LIB_PATH) \
		-L$(OPENCV_3RDPARTY_LIB_PATH) \
		-L$(NNCASE_LIB_PATH) \
		-L$(MPP_SRC_DIR)/userapps/lib \
		-Wl,--start-group \
		-lNncase.Runtime.Native \
		-lfunctional_k230 \
		-lnncase.rt_modules.k230 \
		-lbuffer_management -lsensor -lt_mxml_c -lcommon -lcameric_reg_drv -lcipher -lvdec -ldpu -lvvi -lvirtual_hal -lbinder -ldma -lcmd_buffer -lt_database_c -l3a -lebase -ladec -lvicap -lstart_engine -lvo -lvb -lt_json_c -lcam_caldb -lpm -lfft -lt_common_c -lsys -laenc -lcam_engine -lcameric_drv -lhal -lisi -lao -lvideo_in -lai -lconnector -lswitch -lisp_drv -lfpga -loslayer -lvenc -lcam_device -lauto_ctrol -lvdss \
		$(OPENCV_LIBS) \
		-Wl,--end-group

clean:
	echo "${PWD}/Makefile clean"
	-rm -rf $(BIN) $(ANALYZER_BIN)
	-rm -f $(OBJS) $(CPP_OBJS) $(ANALYZER_CPP_OBJS)

.PHONY: all clean

# VISION_CPP_BUILD_RULES_BEGIN
# Explicit build rules for Stage7D vision C++ objects.
# These objects are linked only into sample_motion_analyzer.elf.

VISION_CPP_INCS = -I$(LOCAL_SRC_DIR) \
                  -I$(MPP_INC) \
                  -I$(MPP_COMM_INC) \
                  -I$(MPP_USERAPPS_INC) \
                  -I$(NNCASE_INC) \
                  -I$(GSL_INC) \
                  -I$(OPENCV_INC) \
                  -I$(JPEG_INC)

$(LOCAL_SRC_DIR)/motion_ai_utils.o: $(LOCAL_SRC_DIR)/motion_ai_utils.cpp $(LOCAL_SRC_DIR)/motion_ai_utils.h
	@echo CXX $@
	$(CPP) $(STAGE7D_CXXFLAGS) -mcmodel=medany -mno-relax $(CFLAGS) $(VISION_CPP_INCS) -c $< -o $@

$(LOCAL_SRC_DIR)/motion_ai_base.o: $(LOCAL_SRC_DIR)/motion_ai_base.cpp $(LOCAL_SRC_DIR)/motion_ai_base.h $(LOCAL_SRC_DIR)/motion_ai_utils.h
	@echo CXX $@
	$(CPP) $(STAGE7D_CXXFLAGS) -mcmodel=medany -mno-relax $(CFLAGS) $(VISION_CPP_INCS) -c $< -o $@

$(LOCAL_SRC_DIR)/motion_ob_det.o: $(LOCAL_SRC_DIR)/motion_ob_det.cpp $(LOCAL_SRC_DIR)/motion_ob_det.h $(LOCAL_SRC_DIR)/motion_ai_base.h $(LOCAL_SRC_DIR)/motion_ai_utils.h
	@echo CXX $@
	$(CPP) $(STAGE7D_CXXFLAGS) -mcmodel=medany -mno-relax $(CFLAGS) $(VISION_CPP_INCS) -c $< -o $@

$(LOCAL_SRC_DIR)/motion_vision.o: $(LOCAL_SRC_DIR)/motion_vision.cpp $(LOCAL_SRC_DIR)/motion_vision.h $(LOCAL_SRC_DIR)/motion_ob_det.h
	@echo CXX $@
	$(CPP) $(STAGE7D_CXXFLAGS) -mcmodel=medany -mno-relax $(CFLAGS) $(VISION_CPP_INCS) -c $< -o $@

# VISION_CPP_BUILD_RULES_END

