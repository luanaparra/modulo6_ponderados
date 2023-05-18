[Voltar para o repositório principal :house:](https://github.com/rmnicola/m6-ec-encontros.git)

# Introdução à Robótica Móvel e ROS <!-- omit in toc -->

## Objetivos do encontro
* Introduzir o conceito de robôs móveis e diferenciá-los de braços robóticos.
* Introduzir o conceito de sistemas operacionais e suas principais funções.
* Introduzir o ROS e suas ferramentas para comunicação.
* Setup de ambiente de desenvolvimento e construção do primeiro nó em ROS.

## Conteúdo <!-- omit in toc -->

- [Objetivos do encontro](#objetivos-do-encontro)
- [O que é um robô móvel?](#o-que-é-um-robô-móvel)
  - [AGVs vs AMRs](#agvs-vs-amrs)
- [O que faz um sistema operacional?](#o-que-faz-um-sistema-operacional)
- [Introdução ao Robot Operating System (ROS)](#introdução-ao-robot-operating-system-ros)
  - [Principais características do ROS](#principais-características-do-ros)
  - [Nós em ROS](#nós-em-ros)
  - [Tópicos em ROS](#tópicos-em-ros)
  - [Serviços em ROS](#serviços-em-ros)
- [Instalando o WSL](#instalando-o-wsl)
- [Instalando o ROS2 Humble](#instalando-o-ros2-humble)
- [Configuração de um venv para ROS](#configuração-de-um-venv-para-ros)
- [Criando um publisher simples em ROS](#criando-um-publisher-simples-em-ros)

## O que é um robô móvel?

<img src="https://interlakemecalux.cdnwm.com/documents/20128/500581/M17P1_Los+robots+m%C3%B3viles+pueden+realizar+tareas+de+clasificaci%C3%B3n+de+paquetes.jpg/87baf89b-a5a7-fbb4-05e8-10c5e76fe898?t=1645516167000&e=jpg" alt="Imagem" style="display: block; margin: 0 auto; width: 70%;">
<p style="margin-left: 15%">Imagem retirada de <a href="https://www.interlakemecalux.com/blog/autonomous-mobile-robots">Interlakemecalux</a></p>

Um robô móvel é um tipo de robô que possui a capacidade de se mover autonomamente em um ambiente físico. Diferentemente dos braços robóticos, que são projetados principalmente para manipular objetos e executar tarefas precisas em um local fixo, os robôs móveis são projetados para se locomover em diferentes espaços e realizar uma variedade de tarefas. Eles geralmente são equipados com rodas, esteiras ou pernas para permitir a movimentação e a exploração do ambiente ao seu redor.

Os robôs móveis são utilizados em diversas aplicações, como logística, manufatura, exploração espacial, segurança e serviços. Eles são capazes de navegar em ambientes complexos, evitar obstáculos e interagir com seu entorno. Alguns robôs móveis são controlados remotamente por um operador humano, enquanto outros são capazes de tomar decisões de forma autônoma com base em sensores e algoritmos de inteligência artificial.

Enquanto os braços robóticos são mais adequados para tarefas precisas e repetitivas em um local fixo, os robôs móveis têm a vantagem da mobilidade e adaptabilidade. Eles podem explorar diferentes áreas, transportar objetos de um local para outro, inspecionar espaços remotos e até mesmo interagir com seres humanos de maneira mais dinâmica. Os robôs móveis oferecem flexibilidade e versatilidade em uma ampla gama de aplicações, tornando-se uma ferramenta valiosa na automação e no avanço tecnológico.

### AGVs vs AMRs
<img src="https://www.infineon.com/export/sites/default/_images/application/Robotics/AGV-AMR-graph.png_1838978739.png" alt="Imagem" style="display: block; margin: 0 auto; width: 70%;">
<p style="margin-left: 15%">Imagem retirada de <a href="https://www.infineon.com/cms/en/applications/robotics/mobile-robots/">Infineon</a></p>

Os AGVs são veículos guiados automaticamente que seguem rotas predefinidas por meio de um sistema de orientação, como trilhos, fitas magnéticas ou fios condutores. Eles são projetados para transportar cargas pesadas e volumosas, geralmente em ambientes estruturados e previsíveis. Os AGVs são altamente eficientes em operações repetitivas e padronizadas, como movimentação de materiais em linhas de montagem ou depósitos. No entanto, sua flexibilidade é limitada, uma vez que requerem infraestrutura física para a movimentação.

Por outro lado, os AMRs são robôs móveis autônomos capazes de navegar em ambientes dinâmicos e não estruturados sem a necessidade de infraestrutura prévia. Eles utilizam sensores, como câmeras, lasers e sonares, para mapear o ambiente e tomar decisões em tempo real. Os AMRs são altamente flexíveis e adaptáveis, podendo lidar com diferentes tipos de carga e executar uma variedade de tarefas, desde transporte até a realização de tarefas específicas, como pegar e colocar objetos em prateleiras. Sua capacidade de planejamento e reconfiguração torna-os ideais para ambientes em constante mudança, como armazéns e centros de distribuição.

## O que faz um sistema operacional?

Um sistema operacional moderno desempenha várias funções essenciais para garantir o funcionamento eficiente de um dispositivo eletrônico. Primeiramente, ele é responsável pelo gerenciamento de recursos, como processador, memória, armazenamento e dispositivos de entrada e saída. O sistema operacional aloca e controla o acesso a esses recursos, garantindo que múltiplos programas e aplicativos possam ser executados simultaneamente de forma segura e eficaz.

Além disso, o sistema operacional fornece uma interface entre o usuário e o hardware do dispositivo. Ele permite que os usuários interajam com o computador através de uma interface gráfica intuitiva, facilitando o uso e o acesso às funcionalidades do sistema. Essa interface também inclui recursos de gerenciamento de arquivos, permitindo que os usuários criem, editem, movam e excluam arquivos e pastas de maneira organizada.

Outra função crucial de um sistema operacional moderno é garantir a segurança e a proteção dos dados e do sistema como um todo. Ele implementa mecanismos de controle de acesso e autenticação para proteger informações confidenciais e impedir o acesso não autorizado. Além disso, o sistema operacional executa tarefas de monitoramento em tempo real para detectar e prevenir ameaças, como vírus e malware, e também oferece atualizações de segurança para manter o sistema protegido contra vulnerabilidades conhecidas.

Em resumo, as funções de um sistema operacional moderno englobam o gerenciamento eficiente dos recursos do dispositivo, a disponibilização de uma interface amigável para o usuário e a garantia da segurança dos dados e do sistema como um todo. Essas características fundamentais são essenciais para o bom funcionamento de computadores, smartphones, tablets e outros dispositivos eletrônicos, tornando possível a execução de uma ampla gama de aplicativos e tarefas de maneira segura e conveniente.

## Introdução ao Robot Operating System (ROS)

O Robot Operating System (ROS) é um framework de software aberto que facilita o desenvolvimento e a integração de aplicações robóticas. O ROS fornece uma arquitetura baseada em nós, comunicação por tópicos e serviços, e uma ampla variedade de bibliotecas e ferramentas. É amplamente utilizado na indústria e na academia para desenvolver e testar sistemas robóticos.

### Principais características do ROS

- Arquitetura modular e flexível
- Comunicação interprocessos por meio de tópicos e serviços
- Ferramentas de visualização e simulação
- Bibliotecas para planejamento de movimento, percepção e controle
- Suporte para múltiplas linguagens de programação, como Python e C++

### Nós em ROS
<img src="https://docs.ros.org/en/foxy/_images/Nodes-TopicandService.gif" alt="Imagem" style="display: block; margin: 0 auto; width: 90%;">
<p style="margin-left: 15%">Imagem retirada de <a href="https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html">ROS Documentation</a></p>

Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

### Tópicos em ROS
<img src="https://docs.ros.org/en/foxy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif" alt="Imagem" style="display: block; margin: 0 auto; width: 90%;">
<p style="margin-left: 15%">Imagem retirada de <a href="https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Nodes.html">ROS Documentation</a></p>
ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.
A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.
Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system.

### Serviços em ROS
<img src="https://docs.ros.org/en/foxy/_images/Service-MultipleServiceClient.gif" alt="Imagem" style="display: block; margin: 0 auto; width: 90%;">
<p style="margin-left: 15%">Imagem retirada de <a href="https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Services.html">Infineon</a></p>

## Instalando o WSL

<a href="https://www.youtube.com/watch?v=Dt1x4NBPp-Y">
<img src="https://img.youtube.com/vi/Dt1x4NBPp-Y/hqdefault.jpg" alt="Imagem" style="display: block; margin: 0 auto; width: 70%;">
</a>

Antes de mais nada, certifique-se que o módulo de Windows do WSL foi habilitado (versões mais novas do Windows já vem com o módulo habilitado). Para isso, abra o PowerShell com privilégios administrativos e execute o seguinte comando:

```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

Esse comando faz alterações que requerem a reinicialização do sistema. Reinicie o computador antes de continuar.

Para garantir que a versão do WSL no seu computador é a mais atualizada possível, utilize o comando a seguir:
```powershell
wsl --update
```

A seguir, procure na Microsoft Store por `Ubuntu 22.04`, pois essa é a distribuição adequada para utilizar com o ROS Humble. Para instalá-la, basta clicar no botão "Obter" e aguardar a conclusão da instalação. Ao terminar instalação inicial, um terminal será aberto solicitando que faça as configurações iniciais do seu sistema Ubuntu. ***Não pule esta etapa***, pois é nela que o sistema criará a pasta HOME do seu usuário e o configurará como a conta padrão e pertencente ao grupo de usuários que podem usar o comando `sudo`.

Parabéns! Agora você tem uma versão de Ubuntu instalada dentro do conforto de seu sistema Windows =)

Mas calma, ainda não acabou. Antes de pularmos para a instalação do ROS, vamos rodar alguns comandos que vão garantir que seu Ubuntu está preparado para o desenvolvimento que faremos a seguir.

Primeiro, vamos atualizar o banco de dados de repositórios do apt com:
```bash
sudo apt update
```

A seguir, vamos atualizar todas as aplicações que não estão em sua versão mais nova com:
```bash
sudo apt upgrade
```

Pronto, agora vamos instalar alguns pacotes que vão ser importantes para nosso desenvolvimento. Para começar, vamos instalar um metapacote que agrega as ferramentas necessárias para compilação e desenvolvimento de software. Rode:
```bash
sudo apt install build-essential
```

Agora, vamos garantir que tudo o que vamos precisar de Python está no sistema. Rode:
```bash
sudo apt install python3 python3-pip python3-venv
```

Por fim, vamos instalar algumas outras ferramentas que podem ser úteis. Rode:
```bash
sudo apt install curl software-properties-common
```

Agora sim! Tudo certo para começarmos a instalação do ROS.

## Instalando o ROS2 Humble


Para instalar o ROS, precisamos adicionar novos repositórios ao apt, pois o ROS não se encontra nos repositórios padrão do Ubuntu. Para isso começaremos garantindo que o repositório `universe` está habilitado. Rode:

```bash
sudo apt-add-repository universe
```

A seguir, precisamos baixar uma chave GPG e adicioná-la ao keyring do sistema para poder validar o repositório que vamos adicionar. Rode:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Agora precisamos adicionar o repositório à lista de repositórios. Use:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Como fizemos alterações nos repositórios do apt, precisamos atualizar seu banco de dados novamente. Rode:

```
sudo apt update
```

Pronto! Agora estamos finalmente prontos para instalar a nossa distribuição de ROS. Para facilitar nossa vida, vamos escolher a versão do pacote mais completa, assim não precisaremos nos preocupar se os exemplos e pacotes que vamos precisar já estarão instalados ou não. Rode:

```bash
sudo apt install ros-humble-desktop
```

Essa instalação vai demorar alguns minutos, então tenha paciência =)

Falta apenas uma coisa para termos o poder do ROS em nossas mãos: por padrão, o ROS não adiciona automaticamente todos os executáveis e variáveis de ambiente ao nosso sistema, mas existe um script que faz todo esse setup para nós. Como ninguém tem tempo de ficar dando source nesse script toda vez, rodem esse comando para garantir que tudo vai estar configurado sempre que você abrir o terminal do WSL:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Perfeito! Agora estamos prontos para trabalhar com o ROS2 Humble. Vamos testar?

Abra dois terminais e, para cada um deles vamos rodar um comando. Para o terminal 1:

```bash
ros2 run demo_nodes_cpp talker
```

Para o terminal 2:
```bash
ros2 run demo_nodes_cpp listener
```

Se tudo deu certo, você acabou de ver dois processos totalmente independentes conversando através da interface de comunicação do ROS. Legal, né? Para fechar as instruções necessárias, vamos apenas aprender a rodar nosso exemplo.

## Configuração de um venv para ROS

<a href="https://www.youtube.com/watch?v=7V-QAYvUXM8">
<img src="https://img.youtube.com/vi/7V-QAYvUXM8/hqdefault.jpg" alt="Imagem" style="display: block; margin: 0 auto; width: 70%;">
</a>

## Criando um publisher simples em ROS
<a href="https://www.youtube.com/watch?v=R1ulM5XFQ0I">
<img src="https://img.youtube.com/vi/R1ulM5XFQ0I/hqdefault.jpg" alt="Imagem" style="display: block; margin: 0 auto; width: 70%;">
</a>