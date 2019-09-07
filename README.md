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

1. "/camera/rgb/image_raw":      imagem RGB publicada pela nó da câmera ASTRA
2. "/camera/depth/image_raw":    imagem de profundidade da câmera ASTRA
3. "/zed/left/image_rect_color": imagem RGB da lente esquerda da ZED
4. "/zed/odom":                  mensagem de odometria vinda da câmera ZED

O nó ROS programado em astra_calibrada.cpp vai pegar essas informações e repassar para serem publicadas em 5 tópicos sincronizados, que seriam:

1. "/astra_projetada": nuvem de pontos RGB calibrada da câmera ASTRA
2. "/pixels"         : nuvem de pontos para falar correspondências entre imagens de profundiadade e RGB da ASTRA
3. "/odom2"          : odometria da câmera ZED sincronizada
4. "/zed2"           : imagem RGB da lente esquerda da ZED sincronizada
4. "/astra2"         : imagem RGB da câmera ASTRA sincronizada

Todos os últimos 5 tópicos publicados devem ser gravados no arquivo BAG usando o roslaunch record.launch . É importante gravar neste arquivo os arquivos raw, não comprimidos, para não perder a sincronia a princípio.
Deve haver opção de nome e local de gravação do arquivo para diferenciar um do outro. No script launch já estao todos os dados que devem ser salvos

### Para lancar o sistema

O sistema é lançado em roslaunch, com o arquivo lancar_sistema.launch . Chamando este arquivo se liga a ASTRA e a ZED com seus respectivos nós ROS, e ainda o nó descrito anteriormente para processar e gravar as imagens.

### Para visualização

A nuvem de pontos que o usuário deveria ver seria a que realmente está sendo gravada, vinda no tópico "/astra_projetada".
Quanto a iamgens, fica a critério do desenvolvedor qual tópico, porém deve ser da lente esquerda da câmera ZED.
