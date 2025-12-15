# PR2-Platform 
<div >
    <a href="https://arxiv.org/abs/2409.01559" target="_blank">
    <img src="https://img.shields.io/badge/Paper-arXiv-green" alt="Paper arXiv"></a> 
    <a href="https://www.youtube.com/watch?v=O0xRILrc6nk" target="_blank">
    <img src="https://img.shields.io/badge/Video-Demos-fffc4f" alt="Demos"/></a>
    <a href="https://pr2-humanoid.github.io/" target="_blank">
    <img src="https://img.shields.io/badge/Page-PR2-3083ff" alt="Website"/></a>
</div>

![splash](Docs/assets/teaser.png)

## 📚 Citation

If you use this project in your research, please consider citing:

```bibtex
@article{liu2025pr2,
  title={PR2: A Physics-and Photo-Realistic Humanoid Testbed With Pilot Study in Competition},
  author={Liu, Hangxin and Xie, Qi and Zhang, Zeyu and Yuan, Tao and Wang, Song and Wang, Zaijin and Leng, Xiaokun and Sun, Lining and Zhang, Jingwen and He, Zhicheng and others},
  journal={Journal of Field Robotics},
  year={2025},
  publisher={Wiley Online Library}
}
```

## ✅ System Requirements
 
### 🍀 Pre-Requisites

Before getting started, ensure that the system has met the system requirements [NVIDIA Driver](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html#system-requirements) and has the [NVIDIA Container Toolkit](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html) installed.

 
 
## 🤖 PR2 Environment Setup
⚠️ Isaac Sim Version Notice

PR2-Platform primarily targets Isaac Sim 2023 on the main branch, where all provided examples are fully tested and more stable.

We also provide experimental support for Isaac Sim 5.0:

   * To use Isaac Sim 5.0, please switch to the isaacsim5.0 branch

   * Follow the setup instructions in the README of isaacsim5.0 branch

   * Use the corresponding asset package (data-v0.0.2.zip)

👉 If you want the most stable examples and recommended workflows, please stay on the main branch (Isaac Sim 2023).

### 🚀 Clone PR2-Platform repo
   ```bash
   git clone git@github.com:pr2-humanoid/PR2-Platform.git
   cd PR2-Platform/PR2
   ```
   By default, you are on the main branch (Isaac Sim 2023).
   
### 🏡 Download the Asset
   Use the following commands to download and place the asset in the PR2-Platform/PR2/data directory. 
   ```bash
   wget https://github.com/pr2-humanoid/PR2-Platform/releases/download/v0.1.0/data-v0.0.1.zip
   unzip data-v0.0.1.zip -d . 
   rm -rf data-v0.0.1.zip
   ```
      
### 🐳 Setting Up Docker

1. **Build Docker Image**
   Run the following command:
   ```bash
   bash docker/script/build-pr2.sh
   ```

2. **Run the Docker Image**
   To set up the PR2 environment, execute:
   ```bash
   bash docker/script/run-pr2.sh
   ```
   This script binds your local repository directory to `/PR2/` inside Docker and initiates a new bash shell.

### 📹 Running a Demo

Once you are in the Docker environment, you can start a demo with:

```bash
bash examples/launch_task.sh <task-id>
```
Replace `<task-id>` with an integer between 1 and 6 to select the specific demo task you want to present.

### 💯 Test Your Solution

To test your TaskSolver, execute:

```bash
bash submission/launch_task.sh <task-id>
```
Replace `<task-id>` with an integer between 1 and 6 to select the specific task you want to test.

## ⭐ Contributing to PR2-Platform

We warmly welcome contributions from the community to help enhance and refine this platform for the benefit of all users. Whether it’s through bug reports, feature requests, or direct code contributions, your input is invaluable to us. If you find this project beneficial, we would greatly appreciate your follow and a star.
