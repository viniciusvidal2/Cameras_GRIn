# HANDSET

Esse branch é especifico para o software que irá embarcado na NVida Jetson. Ele possui, dentre dependencias mais simples, as principais dependências:

> Driver da NVidia e CUDA.
> SDK da ZED;
> Pacote ROS para a ZED;
> Pacote ROS para a ASTRA.

## Driver da NVidia e CUDA

Ambos são pré-requisito para a câmera ZED funcionar. Em máquinas que não são ARM, a experiência mostrou que cada placa funciona melhor com uma versão específica para o driver e para o CUDA. Isso será pedido pelo SDK da ZED.

## SDK da ZED

Já deve ter sido instalado anteirormente, é fundamental casar este SDK com o pacote ROS respectivo.

O link https://www.stereolabs.com/developers/release/#sdkdownloads_anchor contém as versões disponíveis para Jetson atualmente. É pré-requisito para o pacote ROS da ZED

## Pacote ROS para a ZED

Aqui se encontra a maior dificuldade da instalação até o momento. O repositório do Github para clonar o pacote está no link https://github.com/stereolabs/zed-ros-wrapper .

Existem diversos commits no repositório, e pela prática, algum deles deve casar com as versões de SDK e drivers instalados anteriormente.

## Pacote ROS para ASTRA

Para a ASTRA não há dependência com nenhum dos itens anteriores, e seu pacote consegue ser instalado sequindo os links a seguir:

1. https://github.com/orbbec/ros_astra_camera para instalar o primeiro pacote ou http://wiki.ros.org/astra_camera
2. https://github.com/orbbec/ros_astra_launch ou http://wiki.ros.org/astra_launch para o segundo pacote

## Oque é importante

Aqui neste repositório está o nó que lê os dados de astra e zed e os publica em novos tópicos, que devem ser vistos pelo usuário ou salvos em arquivo bag de ROS. Os tópicos lidos são:



