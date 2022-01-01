#######################
# SDCC Makefile for making a hexfile from all .C files in this directory,
# and specified directories.
#
# Output files are created in directory './SDCC'.
# Target file for STM8 programming is ./SDCC/main.ihx
#######################


# define compiler path (if not in PATH), and flags
CC               = sdcc
LD               = sdcc
OPTIMIZE         = 
CFLAGS           = -mstm8 --out-fmt-elf --debug --std-sdcc99 --std-c99 $(OPTIMIZE)
LFLAGS           = -mstm8 --debug -lstm8 --out-fmt-elf

# set output folder and target name
OUTPUT_DIR       = SDCC
TARGET           = $(OUTPUT_DIR)/main.elf

# find all -c and .h in specified directories PRJ_DIRS
PRJ_SRC_DIR      = .
PRJ_INC_DIR      = $(PRJ_SRC_DIR)
PRJ_SOURCE       = $(foreach d, $(PRJ_SRC_DIR), $(wildcard $(d)/*.c))
PRJ_HEADER       = $(foreach d, $(PRJ_INC_DIR), $(wildcard $(d)/*.h))
PRJ_OBJECTS      = $(addprefix $(OUTPUT_DIR)/, $(notdir $(PRJ_SOURCE:.c=.rel)))

# concat all project files
SRC_DIR          = $(PRJ_SRC_DIR)
INC_DIR          = $(PRJ_INC_DIR)
SOURCE           = $(PRJ_SOURCE)
HEADER           = $(PRJ_HEADER)
OBJECTS          = $(PRJ_OBJECTS)

# set compiler include paths
INCLUDE          = $(foreach d, $(INC_DIR), $(addprefix -I, $(d)))

# set make search paths
vpath %.c $(SRC_DIR)
vpath %.h $(INC_DIR)

# debug: print variable and stop
#$(error variable is [${INC_DIR}])


########
# dependencies & make instructions
########

.PHONY: clean all default

.PRECIOUS: $(TARGET) $(OBJECTS)

default: $(OUTPUT_DIR) $(TARGET)

all: default

# create output folder
$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)
	rm -fr -- -p

# link target
$(TARGET) : $(OBJECTS)
	$(LD) $(LFLAGS) -o $@ $(OBJECTS)

# compile objects
$(OBJECTS) : $(SOURCE) $(HEADER)
$(OUTPUT_DIR)/%.rel : %.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

# clean up
clean:
	rm -fr $(OUTPUT_DIR)/*

#EOF
