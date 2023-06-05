# Atividade 4: Backend para transmissão e armazenamento de imagens

## Enunciado

Desenvolva o software de um backend capaz de receber imagens e armazená-las adequadamente. Não há restrições com relação à tecnologia utilizada.

## Padrão de qualidade

Para esta atividade, espera-se a capacidade demonstrável de desenvolvimento de um backend capaz de interagir (não necessariamente em tempo real) com um sistema de visão computacional. A entrega deve ser um vídeo demonstrando o funcionamento do projeto, um texto conciso descrevendo como foi feita a implementação e o link para o repositório público no github onde foi feita a implementação.

1. Setup das rotas do backend. (peso 1)
2. Manipulação adequada da imagem de acordo com a estratégia escolhida (arquivos completos, frames) e integração com a rota do backend. (peso 2)
3. Armazenamento adequado da imagem. (peso 2)
4. Explicação coerente e concisa da implementação (min 250 caracteres e máximo 1500); (peso 3)
5. Congruência entre o que foi escrito e o código disposto no repositório do github; (peso 2)

## Explicação do exercício

A implementação realizada utilizou o modelo YOLO (You Only Look Once) como uma arquitetura de detecção de objetos. O YOLO é conhecido por sua eficiência e capacidade de detectar objetos em tempo real. Dessa maneira, no notebook trainponderada.ipynb foi necessário instalar as bibliotecas necessárias. Utilizei o comando !pip install ultralytics para instalar a biblioteca Ultralytics, que fornece a implementação do YOLO. Também utilizei o comando !pip install roboflow para instalar a biblioteca Roboflow, que permite o download e manipulação de conjuntos de dados.

Para o treinamento do modelo com o dataset disponibilizado pela Roboflow, utilizei a biblioteca roboflow para fazer o download do dataset. Através das linhas de código fornecidas, criei uma instância da classe Roboflow passando a chave de API correspondente à conta do Roboflow utilizada. Em seguida, acessei o projeto e versão específicos do dataset de rachaduras em paredes de concreto e realizei o download utilizando o método download(). Essa etapa é fundamental para fornecer ao modelo exemplos relevantes para o contexto do problema a ser resolvido.

Para o setup do modelo pré-treinado, utilizei a função YOLO() passando o caminho para o arquivo de pesos pré-treinados best.pt. Esse arquivo contém os parâmetros aprendidos durante o treinamento do modelo em um conjunto de dados de referência. Essa etapa permite utilizar o modelo pré-treinado como uma base sólida para a detecção de objetos.

No código principal, utilizei a função VideoCapture() do OpenCV para capturar o vídeo da webcam em tempo real. Em um loop contínuo, cada quadro (frame) do vídeo é lido utilizando a função read() do objeto de captura. Em seguida, utilizei o método predict() do modelo YOLO para realizar a detecção de objetos no quadro. O parâmetro conf foi configurado com um valor de 0.6 para definir a confiança mínima necessária para considerar uma detecção válida.

Após a detecção, utilizei a função imshow() do OpenCV para exibir o quadro com as detecções. A função plot() do objeto resultante da detecção foi utilizada para desenhar as caixas delimitadoras e as classes dos objetos identificados.

Por fim, adicionei uma condição para sair do loop e após sair do loop, liberei os recursos da captura de vídeo e fechei todas as janelas abertas pelo OpenCV.

Vídeo: https://drive.google.com/file/d/17y9viwjcwfArb1VLutEdFyHurvBJp7JT/view?usp=sharing
