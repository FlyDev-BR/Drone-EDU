# EduDrone: Mini Drone Educacional com ArduPilot

<p align="center">
  <img src="https://img.shields.io/badge/Foco-Educação%20STEM-blue" alt="Foco do Projeto">
  <img src="https://img.shields.io/badge/Plataforma-ArduPilot-orange" alt="Plataforma ArduPilot">
  <img src="https://img.shields.io/badge/Linguagem-Python%20%7C%20Blockly-green" alt="Linguagens">
  <img src="https://img.shields.io/github/license/SEU_USUARIO/SEU_REPOSITORIO" alt="Licença">
</p>

> Um projeto de drone educacional, aberto e de baixo custo, desenvolvido para introduzir estudantes de todas as idades aos conceitos de programação e robótica de forma divertida e interativa.

---

## 🚀 O Projeto em Ação

<p align="center">
  <strong>Programando uma Missão com Blocos (Blockly)</strong><br>
  <img src="(https://github.com/FlyDev-BR/Drone-EDU/blob/47cf6440a580923d6b5cceeb5728a1ae659c0990/V%C3%ADdeo%20sem%20t%C3%ADtulo%20%E2%80%90%20Feito%20com%20o%20Clipchamp%20(1).gif)" alt="Demonstração da interface Blockly" width="600"/>
  <em>Interface intuitiva que permite a criação de missões de voo autônomas arrastando e soltando blocos. Perfeito para iniciantes!</em>
</p>

<p align="center">
  <strong>Controlando o Drone Manualmente com Python</strong><br>
  <img src="URL_DO_SEU_GIF_DO_CONTROLE_PYTHON" alt="Demonstração do controle manual com Python" width="600"/>
  <em>Script em Python que permite o controle manual do drone via teclado, ideal para entender a comunicação e controle em tempo real.</em>
</p>

## 📋 Índice

- [Visão Geral](#-visão-geral)
- [Principais Recursos](#-principais-recursos)
- [Atividades Pedagógicas Sugeridas](#-atividades-pedagógicas-sugeridas)
- [Tecnologias Utilizadas](#-tecnologias-utilizadas)
- [Como Começar (Guia de Instalação)](#-como-começar-guia-de-instalação)
- [Como Contribuir](#-como-contribuir)

## 💡 Visão Geral

O **EduDrone** foi criado para desmistificar a tecnologia de drones e torná-la uma ferramenta de aprendizado acessível para escolas e cursos de robótica. Baseado na robusta plataforma **ArduPilot**, este projeto oferece duas formas de interação:

1.  **Programação Visual com Blockly:** Permite que alunos sem experiência prévia em programação criem lógicas complexas e missões de voo de forma visual e intuitiva.
2.  **Controle com Python:** Oferece a estudantes mais avançados a oportunidade de interagir diretamente com o drone, enviando comandos e lendo dados de telemetria, explorando conceitos de programação textual e APIs.

O objetivo é fornecer uma plataforma completa que cresce junto com o conhecimento do aluno.

## ✨ Principais Recursos

- ✅ **Plataforma Aberta:** Hardware de baixo custo e software totalmente open-source.
- 🧱 **Programação em Blocos:** Interface baseada em Blockly para planejamento de missões sem a necessidade de escrever código.
- 🐍 **Controle via Python:** Scripts prontos para controle manual e exemplos de como criar suas próprias aplicações.
- 🚀 **Confiabilidade ArduPilot:** Utiliza um dos mais avançados e seguros firmwares de voo do mundo.
- 🔐 **Design Compacto e Seguro:** Projetado para ser pequeno e leve, ideal para ambientes internos e de sala de aula.

## 🎓 Atividades Pedagógicas Sugeridas

Este projeto pode ser usado em diversas atividades em sala de aula:

1.  **Iniciante: O Primeiro Voo**
    - **Ferramenta:** Controle Manual com Python.
    - **Objetivo:** Entender os eixos de movimento de um drone (roll, pitch, yaw) e executar comandos básicos como decolar, mover e pousar.

2.  **Intermediário: Desenhando Formas Geométricas**
    - **Ferramenta:** Blockly.
    - **Objetivo:** Usar laços e comandos de movimento para fazer o drone voar em padrões geométricos (quadrado, triângulo), ensinando lógica de programação e coordenadas cartesianas.

3.  **Avançado: Missão de "Entrega"**
    - **Ferramenta:** Blockly ou Python.
    - **Objetivo:** Programar o drone para decolar de um ponto A, voar até um ponto B, esperar (simulando uma entrega) e retornar ao ponto de partida, introduzindo o conceito de missões autônomas completas.

## 💻 Tecnologias Utilizadas

| Área                  | Tecnologia/Software                               |
| --------------------- | ------------------------------------------------- |
| **Firmware** | [ArduPilot (Copter)](https://ardupilot.org/copter/) |
| **Controle Manual** | Python 3, [DroneKit](https://dronekit.io/) ou [Pymavlink](https://mavlink.io/en/getting_started/python.html) |
| **Programação Visual** | [Google Blockly](https://developers.google.com/blockly) |
| **Comunicação** | Protocolo MAVLink                                 |
| **Modelagem 3D** | Fusion 360 / Onshape / FreeCAD |

## 🚀 Como Começar (Guia de Instalação)

### Pré-requisitos

- Python 3.8 ou superior.
- Git.
- (Opcional) Software de Estação de Controle como [Mission Planner](https://ardupilot.org/planner/) ou [QGroundControl](http://qgroundcontrol.com/) para configuração inicial do ArduPilot.

### Instalação e Uso

1.  **Clone este repositório:**
    ```sh
    git clone [https://github.com/FlyDev-BR/Drone-EDU.git](https://github.com/FlyDev-BR/Drone-EDU.git)
    cd Drone-EDU/
	
    ```

2.  **Para usar o Controle Manual e de Programação em Python:**
    ```sh
    cd fly-programation-test/Aplicação_Python/
    pip install -r requirements.txt
    python python main.py 
    ```

## 🙌 Como Contribuir

Sua ajuda é muito bem-vinda! Se você é um professor com ideias para novas atividades, um desenvolvedor com sugestões de melhoria ou um aluno que encontrou um bug, sinta-se à vontade para:

1.  Fazer um "Fork" do projeto.
2.  Criar uma nova "Branch" para sua funcionalidade.
3.  Enviar um "Pull Request".

Você também pode abrir uma "Issue" para relatar problemas ou sugerir novas funcionalidades.
