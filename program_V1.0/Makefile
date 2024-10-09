SYSTEM     		= x86-64_linux
LIBFORMAT  		= static_pic

#------------------------------------------------------------
#------------------------------------------------------------

CONCORDEDIR     = /home/kerali/Concorde/concorde
CPLEXINCDIR   	= /home/ibm/cplex-studio/22.1.1/cplex/include
CONCERINCTDIR   = /home/ibm/cplex-studio/22.1.1/concert/include

# ---------------------------------------------------------------------
# Compiler selection 
# ---------------------------------------------------------------------

CXX  			= g++ -O0 -g

# ---------------------------------------------------------------------
# Compiler options 
# ---------------------------------------------------------------------

COPT  			= -m64 -fPIC

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

CLNDIRS 		= -L$(CONCORDEDIR) -L$(CPLEXDIR)/lib -L$(CONCERTDIR)/lib
CLNFLAGS 		= -lconcorde -lconcert -lilocplex -lcplex -lm -lpthread -ldl

CPPFLAGS 		= -I$(CPLEXDIR)/include -I$(CONCERTDIR)/include -I$(CONCORDEDIR) -Iinclude
CFLAGS 			= $(COPT) $(CPPFLAGS)
CXXFLAGS 		= $(CFLAGS) -std=c++11

# Directories
SRCDIR := src
BUILDDIR := build
BINDIR := bin

# List of source files and corresponding object files
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
OBJECTS := $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.o,$(SOURCES))

# Executable name
EXECUTABLE := $(BINDIR)/main

# Main rule to build the executable
all: $(EXECUTABLE)

# Build the executable from object files
$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(CFLAGS) $^ -g -o $@ $(CLNDIRS) $(CLNFLAGS)

# Build each object file from its corresponding source file
$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) -c $(CFLAGS) $< -g -o $@

# Clean the build directory
clean:
	rm -f $(BUILDDIR)/*.o $(EXECUTABLE)
