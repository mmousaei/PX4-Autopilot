CFLAG = -fPIC  -I./src  -lm
SFLAG = -shared -fPIC -lm
LFLAG = -L/home/mohammad/workspace/px4_tilt_test/PX4-Autopilot/src/modules/control_allocator/ControlAllocation/alglib-cpp -Wl,-rpath,/home/mohammad/workspace/px4_tilt_test/PX4-Autopilot/src/modules/control_allocator/ControlAllocation/alglib-cpp -lalglib

PATTERN = $(addsuffix /*.cpp, ./src)
OBJS   := $(patsubst %.cpp,%.o,$(wildcard ${PATTERN}))
LIB = libalglib.so
TST = ./tests/test_c.o
EXE = test_c

%.o: %.cpp
	${CXX} ${CFLAG} -c $< -o $*.o

all: ${OBJS}
	${CXX} ${OBJS} -o ${LIB} ${SFLAG}

test: ${TST}
	${CXX} ${TST} -o ${EXE} ${LFLAG}

clean:
	rm ${OBJS} || continue
	rm ${LIB} || continue
	rm ${TST} || continue
	rm ${EXE} || continue
