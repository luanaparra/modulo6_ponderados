#! /usr/bin/env python3

class EmptyQueueException(Exception):
    pass

class ElementoFila():

    def __init__(self, valor, proximo):
        self.valor = valor
        self.proximo = proximo


class Fila():

    def __init__(self):
        self.primeiro = None
        self.ultimo = None
        self.qtd = 0

    def push(self, valor):
        if self.qtd == 0:
            novo_elemento = ElementoFila(valor=valor, proximo=None)
            self.ultimo = novo_elemento
            self.primeiro = novo_elemento
        else:
            novo_elemento = ElementoFila(valor=valor, proximo=None)
            self.ultimo.proximo = novo_elemento
            self.ultimo = novo_elemento
        self.qtd += 1

    def pop(self):
        if self.qtd == 0:
            raise EmptyQueueException("A fila esta vazia, mermao")
        valor_saida = self.primeiro.valor
        if self.qtd == 1:
            self.primeiro = None
            self.ultimo = None
            self.qtd -= 1
            return valor_saida
        self.primeiro = self.primeiro.proximo
        self.qtd -= 1
        return valor_saida

    def __repr__(self):
        if self.qtd == 0:
            return "Fila vazia"
        return f"Fila({self.qtd} | {self.primeiro.valor} -> {self.ultimo.valor})"


def main():
    f = Fila()
    print(f)
    f.push(3)
    print(f)
    f.push(4)
    print(f)
    f.push(5)
    print(f)
    f.pop()
    print(f)
    f.pop()
    print(f)
    f.pop()
    print(f)


if __name__ == "__main__":
    main()


