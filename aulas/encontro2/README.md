[Voltar para o repositório principal :house:](https://github.com/rmnicola/m6-ec-encontros.git)

# Tipos abstratos de dados <!-- omit in toc -->

## Objetivos do encontro
* Responder o que são tipos abstratos de dados e para que servem.
* Demonstrar a implementação de estruturas de dados comuns como Filas, Pilhas e Listas ligadas.
* Desenvolver o conhecimento de programação orientada a objetos.

## Conteúdo <!-- omit in toc -->

- [Objetivos do encontro](#objetivos-do-encontro)
- [Tipos abstratos de dados](#tipos-abstratos-de-dados)
- [Filas](#filas)
- [Pilhas](#pilhas)
- [Implementação de filas usando listas](#implementação-de-filas-usando-listas)
- [Implementação de filas usando listas ligadas](#implementação-de-filas-usando-listas-ligadas)
- [Implementação de filas usando deque](#implementação-de-filas-usando-deque)

## Tipos abstratos de dados

<img src="https://runestone.academy/ns/books/published/cppds/_images/adt.png" alt="Imagem" style="display: block; margin: 0 auto; width: 40%;">
<p style="margin-left: 25%">Imagem retirada de <a href="https://runestone.academy/ns/books/published/cppds/Introduction/WhyStudyDataStructuresandAbstractDataTypes.html">Runestone Academy</a></p>

Tipos abstratos de dados (TADs) são um conceito chave na ciência da computação que descreve uma abstração de dados, independentemente de sua representação concreta em um programa. Eles definem uma estrutura de dados específica e as operações que podem ser realizadas sobre ela, ocultando os detalhes internos de implementação. Os TADs servem para fornecer uma interface clara e bem definida para manipular dados, permitindo que os desenvolvedores usem essas estruturas de forma eficiente e organizada.

Ao utilizar TADs, os programadores podem se concentrar na lógica do problema em vez de se preocupar com os detalhes de implementação. Isso promove a modularidade e o encapsulamento, permitindo que diferentes partes de um programa interajam por meio de interfaces bem definidas. Os TADs também fornecem uma abstração de alto nível, permitindo que os desenvolvedores trabalhem com estruturas de dados complexas de maneira mais simples e compreensível.

Outra vantagem dos TADs é que eles facilitam a reutilização de código. Uma vez que a implementação interna de um TAD é oculta, os programas podem ser facilmente modificados ou atualizados sem afetar o restante do sistema. Além disso, ao usar TADs padronizados, como listas, filas, pilhas e árvores, os desenvolvedores podem aproveitar bibliotecas existentes e algoritmos otimizados, economizando tempo e esforço no desenvolvimento.

Em resumo, os tipos abstratos de dados são estruturas de dados encapsuladas que fornecem uma interface clara e bem definida para manipular dados. Eles servem para facilitar o desenvolvimento de programas, promovendo a modularidade, o encapsulamento e a reutilização de código. Ao usar TADs, os programadores podem se concentrar na lógica do problema em vez dos detalhes de implementação, resultando em um código mais organizado, compreensível e eficiente.

## Filas

Uma fila é uma estrutura de dados linear que armazena elementos em uma ordem específica, seguindo o princípio de First-In-First-Out (FIFO). Isso significa que o primeiro elemento adicionado à fila será o primeiro a ser removido. As duas principais operações em uma fila são:

**Enfileirar (enqueue)**: adicionar um elemento ao final da fila
<img src="https://camo.githubusercontent.com/5a38160aa4fb98e74c1a0ad5e17d69787d0a0ef00697737bcdf67d1635d69d0b/687474703a2f2f7777772e746861676f6d697a65722e636f6d2f696d672f51756575654164642e676966" alt="Imagem" style="display: block; margin: 0 auto; width: 70%;">
<p style="margin-left: 25%">Imagem retirada de <a href="https://github.com/mary-tkachenko/stack-and-queues">Stacks and Queues</a></p>

**Desenfileirar (dequeue)**: remover o elemento do início da fila
<img src="https://camo.githubusercontent.com/dfc9537ddc4180bbe7d53b555ac1da70b0e8460c545cea9e78ede937eeab114c/687474703a2f2f7777772e746861676f6d697a65722e636f6d2f696d672f517565756552656d6f76652e676966" alt="Imagem" style="display: block; margin: 0 auto; width: 70%;">
<p style="margin-left: 25%">Imagem retirada de <a href="https://github.com/mary-tkachenko/stack-and-queues">Stacks and Queues</a></p>

## Pilhas

Uma pilha é um tipo abstrato de dados que segue a estrutura LIFO (Last-In, First-Out), onde o último elemento inserido é o primeiro a ser removido. As principais operações em uma pilha são: "push" para adicionar um elemento ao topo da pilha, "pop" para remover o elemento do topo da pilha e retorná-lo, e "peek" para acessar o elemento no topo da pilha sem removê-lo. Além disso, é comum ter o método "isEmpty" para verificar se a pilha está vazia e "size" para obter o número de elementos presentes na pilha. As pilhas são amplamente utilizadas em algoritmos de processamento de dados, como avaliação de expressões matemáticas, controle de histórico de navegação em navegadores e gerenciamento de chamadas de função em sistemas operacionais.

As duas principais operações com pilhas são:

**Push**: adiciona um novo elemento no topo da pilha
<img src="https://camo.githubusercontent.com/f1431aa3d59e26c83fabfb0310a03f63c118be03d9642bc8f2612cb5d0f9a53e/687474703a2f2f7777772e746861676f6d697a65722e636f6d2f696d672f537461636b507573682e676966" alt="Imagem" style="display: block; margin: 0 auto; width: 80%;">
<p style="margin-left: 25%">Imagem retirada de <a href="https://github.com/mary-tkachenko/stack-and-queues">Stacks and Queues</a></p>

**Pop**: retira um elemento do topo da pilha
<img src="https://camo.githubusercontent.com/d8d3d929a18cdd3076e99586e494ca194e6b8414692e7c5c1a480c765616b56c/687474703a2f2f7777772e746861676f6d697a65722e636f6d2f696d672f537461636b506f702e676966" alt="Imagem" style="display: block; margin: 0 auto; width: 60%;">
<p style="margin-left: 25%">Imagem retirada de <a href="https://github.com/mary-tkachenko/stack-and-queues">Stacks and Queues</a></p>


## Implementação de filas usando listas

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/nBv1Q_bdbTw/hqdefault.jpg)](https://www.youtube.com/watch?v=nBv1Q_bdbTw)

## Implementação de filas usando listas ligadas

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/1oFL12wy9Yw/hqdefault.jpg)](https://www.youtube.com/watch?v=1oFL12wy9Yw)

## Implementação de filas usando deque

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/68DhAuEBVP8/hqdefault.jpg)](https://www.youtube.com/watch?v=68DhAuEBVP8)

