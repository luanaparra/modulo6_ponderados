#! /usr/bin/env python3

from collections import deque


def main():
    fila = deque()
    print(fila)
    fila.append(1)
    print(fila)
    fila.append(2)
    print(fila)
    fila.append(3)
    print(fila)
    fila.popleft()
    print(fila)
    fila.popleft()
    print(fila)
    fila.popleft()
    print(fila)


if __name__ == "__main__":
    main()
