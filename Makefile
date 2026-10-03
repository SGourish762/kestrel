# kestrel - real-time vehicle telemetry engine
CC      ?= cc
STD     := -std=c11
WARN    := -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion -Wstrict-prototypes -Werror
OPT     ?= -O2 -g
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
  DEFS  := -D_GNU_SOURCE
endif
CFLAGS  += $(STD) $(WARN) $(OPT) $(DEFS) -Iinclude
LDLIBS  += -lpthread -lm

BUILD   ?= build
LIB_SRC := $(wildcard src/*.c)
LIB_OBJ := $(LIB_SRC:src/%.c=$(BUILD)/obj/%.o)
LIB     := $(BUILD)/libkestrel.a

BENCHES := $(patsubst bench/%.c,$(BUILD)/%,$(wildcard bench/*.c))

.PHONY: all test bench asan tsan clean sample run
all: $(BUILD)/kestrel $(BUILD)/test_kestrel $(BENCHES)

$(BUILD)/obj/%.o: src/%.c $(wildcard include/kestrel/*.h) | $(BUILD)/obj
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB): $(LIB_OBJ)
	$(AR) rcs $@ $^

$(BUILD)/kestrel: tools/kestrel.c $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) $< $(LIB) $(LDLIBS) -o $@

$(BUILD)/test_kestrel: tests/test_kestrel.c tests/test.h $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) $< $(LIB) $(LDLIBS) -o $@

$(BUILD)/bench_%: bench/bench_%.c $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) $< $(LIB) $(LDLIBS) -o $@

$(BUILD)/obj:
	mkdir -p $@

test: $(BUILD)/test_kestrel
	./$(BUILD)/test_kestrel

bench: $(BENCHES)
	@for b in $(BENCHES); do echo "== $$b"; ./$$b || exit 1; done

run: $(BUILD)/kestrel
	./$(BUILD)/kestrel --duration 10 --faults 0.01 -v

sample: $(BUILD)/kestrel
	./$(BUILD)/kestrel --gen data/sample.candump --duration 30 --faults 0.01

# Sanitizer builds: memory errors + UB, and data races.
asan:
	$(MAKE) BUILD=build-asan OPT="-O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=all" \
	        LDFLAGS="-fsanitize=address,undefined" test
tsan:
	$(MAKE) BUILD=build-tsan OPT="-O1 -g -fsanitize=thread" LDFLAGS="-fsanitize=thread" test

clean:
	rm -rf build build-asan build-tsan
