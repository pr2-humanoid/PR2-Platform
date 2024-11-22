# PR2-Platform 
<div >
    <a href="https://arxiv.org/abs/2409.01559" target="_blank">
    <img src="https://img.shields.io/badge/Paper-arXiv-green" alt="Paper arXiv"></a> 
    <a href="https://www.youtube.com/watch?v=VycZ9Po9hNg" target="_blank">
    <img src="https://img.shields.io/badge/Video-Demos-fffc4f" alt="Demos"/></a>
    <a href="https://pr2-humanoid.github.io/" target="_blank">
    <img src="https://img.shields.io/badge/Page-PR2-3083ff" alt="Website"/></a>
</div>

![splash](Docs/assets/teaser.png)

## ✅ System Requirements

The following steps are taken from the NVIDIA Omniverse Isaac Sim documentation on [container installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html).

### 🍀 Pre-Requisites

Before getting started, ensure that the system has the latest [NVIDIA Driver](https://www.nvidia.com/en-us/drivers/unix/) and the [NVIDIA Container Toolkit](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) installed.


### 📁 Obtaining the Isaac Sim Container
1. **Get access to the Isaac Sim Container by joining the NVIDIA Developer Program credentials**
2. **Generate your [NGC API Key](https://docs.nvidia.com/ngc/ngc-overview/index.html#generating-api-key) to access locked container images from NVIDIA GPU Cloud (NGC).**
   - This step requires you to create an [NGC account](https://ngc.nvidia.com/signin) if you do not already have one.
   - You would also need to install the [NGC CLI](https://org.ngc.nvidia.com/setup/installers/cli) to perform operations from the command line. 
   - Once you have your generated API key and have installed the NGC CLI, you need to log in to NGC from the terminal.
   ```bash
   ngc config set
   ```

3. **Use the command line to login to your NGC Account.**
   ```bash
      docker login nvcr.io
   ```
   - For the username, enter $oauthtoken exactly as shown. It is a special username that is used to authenticate with NGC.
   ```bash
      Username: $oauthtoken
      Password: <Your NGC API Key>
   ```

 
## 🤖 PR2 Environment Setup

### 🚀 Clone PR2-Platform repo
   ```bash
   git clone git@github.com:pr2-humanoid/PR2-Platform.git
   cd PR2-Platform/PR2
   ```
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
 
2. **Start Docker on the Server** 
   Server-side Execution:

   Run the following script to bind your local repository directory to /PR2/ inside the Docker container and start a new bash shell:
   ```
   bash docker/script/run-pr2-host.sh
   ``` 

   Client-side Execution:
   SSH into the docker container:
   ```
   ssh root@<server IP address> -p 2222
   ```

   After entering the password, run the following commands: 
   ```
   cd ../PR2/
   bash docker/script/main-pr2.sh 
   ```

   Access via Web Browser:
   Open a web browser and navigate to the following URL:
   ```
   http://<server IP address>:8211/streaming/webrtc-client?server=<server IP address> 
   ```
   
   NOTE:
   The WebRTC Browser Client may not work with Firefox. Using the Google Chrome or Chromium browser is recommended. 

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
