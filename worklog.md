
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

### TODO

- Desenvolver relatório preliminar (apresentação a 9 de Março)

- Começar o start of art
	- deteção de bolas/objetos/carros/pessoas
	- algoritmos usados
	- ferramenta semi-auto de labeling para deep-learning

- Ler sobre SIFT e SURF