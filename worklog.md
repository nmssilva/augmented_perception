
#### AtlasCar-V2 Augmented Perception
#### Tese de Mestrado - Nuno Silva, nº72708
# Worklog

25/10/17 Quarta
===============

- Manhã no lar a testar o pacote de calibração e descobrir como se lança a interface gráfica
- Reunião com mentores
- Tarde no carro a testar a calibração nos sensores
	- camera nao funcionava, problema na consistencia da imagem. tentei resolver a situação mas ficou em standby

27/10/17 Sexta
==============

- Investigar o KITTI dataset
- Pesquisar outros datasets relevantes

04/11/17 Sábado
===============

- passei a última semana hospitalizado, só tive alta ontem.
- Tentei pegar num computador antigo que tinha para lhe instalar Ubuntu e testar o meu package numa máquina virgem
	- Computador tem problemas na fonte de alimentação (bateria não carrega e desliga subitamente)
	- Desisti do computador e instalar virtualbox no meu PC para poder prosseguir com isso
- Instalei ROS kinetic e estou a tentar correr o meu package a 100%

05/11/17 Domingo
================

- O meu package está com alguns erros na VM que ainda não consegui resolver.
	- Vou deixar isto para outra altura e começar a passar os olhos no kitti dataset para analisar a sua a estrutura de dados...

07/11/17 Terça
==============

- Extração do vídeo do bag como o professor Paulo pediu
- Dum momento para o outro o que tinha feito deixou de funcionar e não sei porquê... 
	- O problema parece ser semelhante ao da VM mas tudo estava a funcionar antes de testar na VM...
	- Maracação de reunião com o Diogo para ele me ajudar com este problema

08/11/17 Quarta
===============

- Investigar mais datasets além do KITTI
- Exploração das diferentes estruturas de dados e comparação dos mesmos
- Reunião com o prof. Paulo e Vitor + José Paulo (jose.paulo@ua.pt)
- Reunião com o Diogo ao fim do dia
	- Não foi possivel encontrar o problema do package de calibração...
	- Os outros problema foram resolvidos (problema num path estático...)

13/11/17 Segunda
================

- Não foi possível trabalhar durante o fim de semana devido a um entrega de um trabalho para outra disciplina
- O Diogo mandou-me um email com uma package de calibração e o package dele (free_space_detection) atualizados
- Estive a simular a calibração com um bag

14/11/17 Terça
==============

- Parte da manhã no carro a testar o package de calibração com sucesso apenas com sensores
- Camera continua com problemas de aquisição de dados no pacote de calibração...

15/11/17 Quarta
===============

- Consegui obter imagens da camera. Solução: mudar packet size e delay no nodelet.cpp do package do pointgrey.
- Ainda não é desta que a camera se liga ao package de calibração. Continua o mesmo erro...

12/02/18 Segunda
================

- "Reboot" do trabalho
- Analíse da estrutura de dados do LaserScan no ROS

16/02/18 Sexta
==============

- Reunião com o prof. Paulo Dias
- Definição dos objetivos principais
- Escolha do template do documento

19/02/18 Segunda
================

- Inicio da implementação de um nó que detecta a bola

22/02/18 Quinta
===============

- Reunião no LAR com o prof. Vítor
- Amostra do desenvolvimento do nó que detecta a bola ao prof. Paulo
- (Finalmente) Melhoria da qualidade da camera
	- Problema: Linux memory buffer demasiado pequeno
	- Resolução: http://www.ptgrey.com/KB/10016

23/02/18 Sexta
==============

- Gravação de um bag com melhor qualidade para usar nos próximos testes
- Melhoramento dos acessos à memória
	- Substituição de ponteiros por cv::Mat

25/02/18 Domingo
================

- Melhoria no nó
	- Bola detectada quase na totalidade
	- Algum ruído á volta na imagem mas fácil de remover

27/02/18 Segunda
================

- Redução do rúido na imagem
- Bounding circle à volta da bola

28/02/18 Terça
==============

- Bounding Box à volta da bola
- Início do desenvolvimento de click and detect

01/03/18 Quinta & 02/03/18 Sexta
================================

- Formação de ROS com prof. Miguel
- Preparação da Apresentação para o dia 09/02/18

05/03/18 Segunda
================

- Tarde no AtlasCar
	- Após um debugging intensivo, descobri (finalmente) porque é que o pacote de calibração não funcionava
	- Problema: faltava um pacote "pointgrey_fl3_ge_28s4_c"
	- Nunca ninguém me disse que este pacote era necessário. O pacote que o Diogo me deu para as PointGrey era outro e sempre funcionou. O pacote de calibração usa outro pacote diferente para as câmeras.
	- Solução: Pacote encontrado [aqui](http://lars.mec.ua.pt/public/LAR%20Projects/Humanoid/2017_JorgeSousa/Code/src/pointgrey_fl3_ge_28s4_c/)
	
06/03/18 Terça
==============

- Desenvolvido launchfile para lançar rviz configurado com as janelas para deteção da bola
- Nó de deteção da bola agora deteta a bola a partir dum clique que determina a cor
	- Desta forma é possivel calibrar a partir duma bola com uma cor arbitrária
- Gravação de rosbag de imagens da camera com a driver do pacote de calibração
	- Necessário para fazer testes no pacote de calibração fora do carro
- Estrutura base do relatório preliminar

07/03/18 Quarta
===============

- Conclusão e entrega do relatório preliminar (estive o dia fora de Aveiro, foi o máximo que pude fazer hoje)

08/03/18 Quinta
===============

- Workshop de ROS

09/03/18 Sexta
===============

- Reunião com prof. Paulo
- Avanço no nó da calibração da camera
	- detecção da bola feita
	- falta publicação do centro da bola
	
12/03/18 Segunda
================

- Avanço no nó de calibração
	- Centro da bola a ser publicado
	- Ficheiros .pcd gerados a partir dos centros detetados
		- Parece existir alguma incoerência nos dados do pcd
		
13/03/18 Terça
==============

- Ficheiro pcd gerado a partir da imagem da camera
- Pointcloud é gerada com o pacote de calibração


14/03/18 Quarta
===============

- Reunião com prof. Paulo e prof. Vitor
- Teste no carro com os sensores para calibração
	- Por alguma razão o Sick LD-MRS400001 não funciona no package	

15/03/18 Quinta
===============

- Teste no carro com os sensores para calibração
	- Calibração com sensores e camera efetuada com sucesso
	- O problema do Sick LD-MRS400001 era ter partes fundamentais comentadas no launchfile da sua driver no pacote de calibração (por alguma razão)
		- Alteração desse mesmo launchfile para funcionar corretamente
	- Gravação de um vídeo para mostrar na apresentação de amanhã
- Conclusão da preparação da apresentação

16/03/18 Sexta
==============

- Apresentações / LAR Meeting
- Início do desenvolvimento do nó de detecção de carros 


19/03/18 Segunda
================

- Experiências com LaserScans para deteção de carros

20/03/18 Terça
==============

- Experiências com Imagem da camera para deteço de carros

21/03/18 Quarta
===============

- Primeira abordagem para deteção de carros
	- Implementação do método de Optical Flow Lucas-Kanade nas imagens de camera
	
22/03/18 Quinta
===============

- Implementação de Template de Matching nas imagens da camera
	- Tracking a ser feito a partir deste método
	- Primeiro protótipo de tracking semi-automático
	
23/03/18 Sexta ~ 26/03/18 Segunda
=================================

- ENEI

27/03/18 Terça
==============

- Template matching guarda os templates adquiridos com uma label dada por input do utilizador


28/03/18 Quarta
===============

- Reunião com prof. Paulo e Vitor
	- Ver MTT do Jorge Almeida
	- Projetar os laserScans na imagem
	- Escrever um ficheiro com uma estrutura de dados do género KITTI
- Implementação do template matching para as frames anteriores

30/03/18 Sexta
==============

- Otimização do algoritmo de template matching
	- Substituição de arrays por queues
	- Backtracking só é feito no ato de salvaguarda
		- Impede que o video pare devido ao processamento destes dados no momento de aquisiço da bounding box
- Implementação de funcionalidade para remover o template / desistir do tracking

02/04/18 Segunda ~ 04/04/18 Quarta
==================================

- Experiências com laserScans com a library [MTT](http://lars.mec.ua.pt/lartk/doc/mtt/html/)

05/04/18 Quinta
===============

- Reunião com prof. Paulo Dias
	- Descoberta do ponto fulcral do problema do MTT: Conversão da pointCloud para Targets
- Início do desenvolvimento do nó de deteção de objetos nos laser scans usando MTT
- Resolvi um problema com este [link](https://github.com/strands-project/strands_3d_mapping/issues/67). Já não me lembro ao certo qual era o problema mas era algo relacionado com pointclouds. Fica aí o link para se no futuro aparecer isso outra vez...


06/04/18 Sexta
==============

- Objetos a serem encontrados com MTT
	- Os objetos são representados pela ligação dos vários pontos da pointcloud
	- Cada objeto tem um ID (numérico) e este é detetado automaticamente
	- O feito o tracking do objeto
		- o ID não se perde na próxima mensagem, este é seguido à medida que o objeto se move
		
09/04/18 Segunda
================

- Criação da estrutura de um protótipo do dataset
- Criação da estrutura de dados das bounding boxes para preencher o dataset
- Nó de labelling cria ficheiros de datasets

10/04/18 Terça
==============

- Criação de um nó de "playback" usando os ficheiros datasets
	- É apresentado um video do bag com as anotações do ficheiro dataset de input

11/04/18 Quarta
===============

- Reunião com prof. Paulo de Vítor
	- Informação das Bounding boxes 3D deve estar nos datasets
	- Projeção dos dados laser na imagem ainda está por fazer
	- Avançar com o documento da tese
		- Esqueleto do documento
			- Secção da calibração com as melhorias e dados
			- Secção a falar sobre MTT, Template Matching, ...
- Datasets incluem agora a label do objeto e um ID
- Melhoria no playback do dataset
	- Bounding boxes mudam cor consoante o label
	- Aparece uma "tag" em cima da bouning box com a Label e o ID do objeto
	
12/04/18 Quinta
===============

- Bounding Box 3D a aparecer no Rviz

13/04/18 Sexta
==============

- Melhoria na filtragem do scanner
	- Desenvolvimento de função para extrair limites da estrada
	

14/04/18 Sábado
===============

- Dataset suporta dados da posição 3D

15/04/18 Domingo
================

- Bounding Box a partir de um clique
- Bounding Box na imagem reajusta o tamanho conforme a distancia obtida pelo MTT

16/04/18 Segunda ~ 20/04/18 Sexta
==================================

- Desenvolvimento da introdução
- Desenvolvimento do state of art
- Desenvolvimento das especificaçes do hardware
- Proposta de Indice

21/04/18 Sábado ~ 22/04/18 Domingo
==================================

- Bug encontrado no MTT: ao selecionar o objeto na imagem, o marker e os valores das posições ficam lentos (os valores não correspondem à realidade...)
- Início da fusão dos laser scans (já que este bug vai demorar algum tempo até o perceber/resolver)

23/04/18 Segunda
================

- Fusão dos laser scans numa única pointcloud


24/04/18 Terça
==============

- Transformação dos laserscans laterais para calibrar a pointcloud

26/04/18 Quinta
===============

- Integração dos laserScans com MTT
- Re-ajuste do algoritmo e tracking para a nova pointcloud
- Tamanho da bounding box fixed


27/04/18 Sexta ~ 07/05/18 Terça
===============================

- Integração dos laserScans com MTT
- Re-ajuste do algoritmo e tracking para a nova pointcloud
- Tamanho da bounding box fixed
