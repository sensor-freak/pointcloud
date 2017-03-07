
include ../../config.mk

CPPFLAGS = $(XML2_CPPFLAGS) $(ZLIB_CPPFLAGS) $(GHT_CPPFLAGS) $(LAZPERF_CPPFLAGS) -I ../../lib
LDFLAGS = $(XML2_LDFLAGS) $(ZLIB_LDFLAGS) $(GHT_LDFLASGS)
CFLAGS += -fPIC

OBJS = \
	pc_li3ds_matrix.o

all: $(LIB_A_LI3DS)

$(LIB_A_LI3DS): $(OBJS)
	ar rs $@ $^

clean:
	@rm -f $(OBJS) $(LIB_A_LI3DS)
	$(MAKE) -C cunit $@

install:
	@echo "No install target in lib"

check:
	$(MAKE) -C cunit $@

