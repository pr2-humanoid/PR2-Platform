# PR2-Platform 
![splash](Docs/assets/teaser.png)

## System Requirements

The following steps are taken from the NVIDIA Omniverse Isaac Sim documentation on [container installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html).

### Pre-Requisites

Before getting started, ensure that the system has the latest [NVIDIA Driver](https://www.nvidia.com/en-us/drivers/unix/) and the [NVIDIA Container Toolkit](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) installed.


### Obtaining the Isaac Sim Container
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

 
## PR2 Environment Setup

### Clone PR2-platform repo
   ```bash
   git clone git@github.com:pr2-humanoid/PR2-Platform.git
   cd PR2-Platform
   ```
### Download the Asset
   Use the following commands to download and place the asset in the PR2/data directory. 
   ```bash
   wget https://github.com/pr2-humanoid/PR2-Platform/releases/download/beta/data-v0.0.1.zip
   unzip data-v0.0.1.zip -d ./PR2/data
   ```
      
### Setting Up Docker

1. **Build Docker Image**
   Run the following command:
   ```bash
   bash docker/scripts/build-dev.sh
   ```

2. **Run the Docker Image**
   To set up the PR2 environment, execute:
   ```bash
   bash docker/scripts/run-dev.sh
   ```
   This script binds your local repository directory to `/PR2/` inside Docker and initiates a new bash shell.

## Running a Demo

Once you are in the Docker environment, you can start a demo with:

```bash
bash examples/launch_task.sh <task-id>
```
Replace `<task-id>` with an integer between 1 and 6 to select the specific demo task you want to present.

