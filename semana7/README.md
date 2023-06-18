# Aplicação com yolo/opencv

Capacidade demonstrável de interagir com imagens utilizando os conceitos de visão computacional e modelos preditivos pré-treinados. E aplicar esse modelo em vídeos ou registros de imagem utilizando de rotas de backend para armazenamento tanto local quanto online das imagens.  

## Tecnologias utilizadas

- **Roboflow** - O Roboflow é uma plataforma online que capacita desenvolvedores a criar seus próprios aplicativos de visão computacional. Ao oferecer uma variedade de ferramentas, o Roboflow simplifica o processo de transformação de imagens em modelos personalizados de visão computacional. Os desenvolvedores podem pré-processar, anotar e preparar conjuntos de dados de imagens para treinar seus modelos de forma personalizada, tornando o desenvolvimento de aplicativos com recursos avançados de visão computacional mais acessível.

- **Ultralytics - YOLOv8** - O YOLO (You Only Look Once), também conhecido como YOLOv8, é uma arquitetura de modelo pré-treinada reconhecida por sua eficácia na detecção de objetos. Ao analisar uma imagem, o YOLOv8 identifica objetos presentes, atribui-lhes uma classe e fornece informações precisas sobre sua localização por meio de "bounding boxes". Sua notável velocidade e precisão o tornam uma escolha popular para aplicativos em tempo real, onde a detecção de objetos precisa ser rápida e eficiente.

- **OpenCV** - OpenCV (Open Source Computer Vision Library) é uma biblioteca de código aberto amplamente adotada para processamento de imagens e visão computacional. Com um conjunto abrangente de funções e algoritmos otimizados, o OpenCV capacita os desenvolvedores a realizar uma variedade de tarefas relacionadas à análise, manipulação e compreensão de imagens e vídeos. Seja reconhecimento de padrões, detecção de objetos ou rastreamento de movimento, o OpenCV é uma ferramenta essencial para soluções de visão computacional.

- **FastAPI** - O FastAPI é um framework moderno e de alto desempenho desenvolvido para a criação de APIs em Python. Projetado para ser fácil de usar, rápido e produtivo, o FastAPI aproveita a anotação de tipos introduzida no Python 3.7+. Essa abordagem permite que os desenvolvedores definam explicitamente os tipos de dados esperados nas rotas da API, proporcionando validação automática dos dados de entrada e retornando mensagens de erro detalhadas quando necessário. Com o FastAPI, é possível construir APIs eficientes e bem estruturadas com facilidade e segurança.

- **SupaBase** - O SupaBase é uma plataforma de desenvolvimento de aplicativos de código aberto que combina um banco de dados PostgreSQL com uma API RESTful pronta para uso. Ao oferecer recursos avançados, como autenticação, armazenamento de arquivos, geração de APIs e websockets em tempo real, o SupaBase fornece uma base sólida para o desenvolvimento de aplicativos modernos. Essa combinação poderosa facilita a criação de aplicativos escaláveis e seguros, agilizando o processo de desenvolvimento e permitindo uma melhor interação com os dados.

#### Observação

Para dar continuidade ao exercício de aplicação com YOLO/OpenCV, utilizei os conhecimentos adquiridos nos exercícios anteriores e para isso, utilizei um dataset pré-existente contendo imagens de rachaduras, que estavam separadas em conjuntos de treino, teste e validação.

## Desenvolvimento

1. Dataset no Roboflow;
2. Instalação dos pacotes e bibliotecas necessários;
3. Treinamento do modelo;
4. Implementação do publisher (envia todos os frames de um vídeo para o nosso sub) e subscriber (recebe cada um dos arquivos de imagem, converte esses arquivos para o formato desejado e estabelece conexão com a rota de backend para armazenamento local de todas as imagens geradas);
5. Criar um bucket no Supabase;
6. Envio das imagens para o SupaBase com FastAPI;

## Rotas - Servidor com FastAPI 

- Rota `/list` - lista todas as imagens do Bucket.

- Rota `/upload` - upload de uma imagem para o servidor.

- Rota `/images` - upload de todas as imagens presentes na pasta `../../../openCV/recebidos` para o servidor.

## Vídeo: 
https://drive.google.com/file/d/17y9viwjcwfArb1VLutEdFyHurvBJp7JT/view?usp=sharing
