# Atividade 3: Processamento de imagens e detecção de objetos

## Enunciado

Desenvolva um script em Python capaz de identificar rachaduras em paredes de concreto. Utilize o [dataset](https://universe.roboflow.com/university-bswxt/crack-bphdr/dataset/2) desenvolvido pela Roboflow. Para o desenvolvimento dessa atividade, recomenda-se o uso de um modelo de detecção de objetos pré-treinado, como o [YoLo](https://github.com/ultralytics/ultralytics). É possível ver um exemplo de como desenvolver um script similar [nesse vídeo](https://www.youtube.com/watch?v=vFGxM2KLs10).

## Padrão de qualidade

Para esta atividade, espera-se a capacidade demonstrável de interagir com imagens utilizando os conceitos de visão computacional e modelos preditivos pré-treinados. A entrega deve ser um vídeo demonstrando o funcionamento do projeto, um texto conciso descrevendo como foi feita a implementação e o link para o repositório público no github onde foi feita a implementação. O código fonte pode ser tanto em formato de de script em Python (`.py`) como em formato de notebook Jupyter (`.ipynb`). A nota se divide em:

1. Setup do modelo pré-treinado utilizado; (peso 2)
2. Retreinamento do modelo para o conjunto de dados disponibilizado; (peso 3)
3. Explicação coerente e concisa da implementação (min 250 caracteres e máximo 1500); (peso 3)
4. Congruência entre o que foi escrito e o código disposto no repositório do github; (peso 2)

## Explicação do exercício

A implementação realizada utilizou o modelo YOLO (You Only Look Once) como uma arquitetura de detecção de objetos. O YOLO é conhecido por sua eficiência e capacidade de detectar objetos em tempo real. Dessa maneira, no notebook trainponderada.ipynb foi necessário instalar as bibliotecas necessárias. Utilizei o comando !pip install ultralytics para instalar a biblioteca Ultralytics, que fornece a implementação do YOLO. Também utilizei o comando !pip install roboflow para instalar a biblioteca Roboflow, que permite o download e manipulação de conjuntos de dados.

Para o treinamento do modelo com o dataset disponibilizado pela Roboflow, utilizei a biblioteca roboflow para fazer o download do dataset. Através das linhas de código fornecidas, criei uma instância da classe Roboflow passando a chave de API correspondente à conta do Roboflow utilizada. Em seguida, acessei o projeto e versão específicos do dataset de rachaduras em paredes de concreto e realizei o download utilizando o método download(). Essa etapa é fundamental para fornecer ao modelo exemplos relevantes para o contexto do problema a ser resolvido.

Para o setup do modelo pré-treinado, utilizei a função YOLO() passando o caminho para o arquivo de pesos pré-treinados best.pt. Esse arquivo contém os parâmetros aprendidos durante o treinamento do modelo em um conjunto de dados de referência. Essa etapa permite utilizar o modelo pré-treinado como uma base sólida para a detecção de objetos.

No código principal, utilizei a função VideoCapture() do OpenCV para capturar o vídeo da webcam em tempo real. Em um loop contínuo, cada quadro (frame) do vídeo é lido utilizando a função read() do objeto de captura. Em seguida, utilizei o método predict() do modelo YOLO para realizar a detecção de objetos no quadro. O parâmetro conf foi configurado com um valor de 0.6 para definir a confiança mínima necessária para considerar uma detecção válida.

Após a detecção, utilizei a função imshow() do OpenCV para exibir o quadro com as detecções. A função plot() do objeto resultante da detecção foi utilizada para desenhar as caixas delimitadoras e as classes dos objetos identificados.

Por fim, adicionei uma condição para sair do loop e após sair do loop, liberei os recursos da captura de vídeo e fechei todas as janelas abertas pelo OpenCV.

Vídeo: https://drive.google.com/file/d/17y9viwjcwfArb1VLutEdFyHurvBJp7JT/view?usp=sharing
