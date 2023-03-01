build:
	g++ src/uart_bridge.cpp -I include/include/ -L include/lib/ -l serial -o uart_bridge

clean:
	rm uart_bridge


run:
	make build && ./uart_bridge

example:
	g++ src/uart_example.cpp -I include/include/ -L include/lib/ -l serial -o uart_bridge