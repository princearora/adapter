
CXXFLAGS = -g -I../src -DTHREADED -Wall
LDFLAGS = 
LIBS = -lm -llabjackusb 

vpath %.hpp ../src
vpath %.cpp ../src
vpath %.hh ../src
vpath %.cc ../src



all: labjack

labjack: labjack.o device_datum.o string_buffer.o client.o server.o adapter.o u3.o labjack_adapter.o \
	service.o logger.o condition.o time_series.o 
	g++ ${CXXFLAGS} ${LDFLAGS} -o labjack $+ ${LIBS}

device_datum.o: device_datum.cpp device_datum.hpp string_buffer.hpp

string_buffer.o: string_buffer.cpp string_buffer.hpp

client.o: client.cpp client.hpp

server.o: server.cpp server.hpp client.hpp

adapter.o: adapter.cpp adapter.hpp server.hpp client.hpp

u3.o: u3.cc u3.hh

labjack.o: labjack.cpp labjack_adapter.hpp adapter.hpp 

labjack_adapter.o: labjack_adapter.cpp labjack_adapter.hpp adapter.hpp server.hpp client.hpp condition.hpp u3.hh

condition.o: condition.cpp condition.hpp device_datum.hpp

logger.o: logger.cpp logger.hpp

service.o: service.cpp service.hpp

time_series.o: time_series.cpp time_series.hpp 

