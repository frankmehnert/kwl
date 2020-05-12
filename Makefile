ASAN = -fsanitize=undefined \
       -fsanitize=address \
       -fsanitize=return \
       -fsanitize=bounds \
       -fsanitize=object-size \
       -static-libasan

kwl: main.cc
	@g++ -g -Wall -Wextra -Os -o $@ $<

clean:
	@rm -f kwl

.PHONY: clean
