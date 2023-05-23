---
layout: post
title:  "Stop Docker container"
date:   2023-04-20 13:51:00 +0700
categories: [docker, linux]
---
## Commands
### for contatiner started by `docker run`
`docker stop $(container_name_or_id)` - send a SIGTERM signal to container, giving it a chance to gracefully shut down. If the container does not stop within a certain timout period (defaul: 10 sec), Docker will forcefully terminate it by sending SIGKILL.

`docker kill $(container_name_or_id)` - directly send SIGKILL to conatainer to forcefully terminate it, which is more brutal than docker stop, since it does not allow the container to perform necessary shutdown tasks. So it is only used if the container is unresponsive or need an immediate termination.


### for multiple containers started by `docker-compose up`
`docker-compose -f $(docker_compose_filename.yml) down` - stop and remove running containers, as well as any networks created. If one wants to remove all volumes too, add `-v` flag. There are also many other flags, refer to reference section.

If the filename is not specified, it is defaulted as "docker-compose.yml"

`docker-compose -f $(dockerc_compose_filename.yml) stop` - stop running containers, but would not remove them.

## Possible issues & solutions
### Issue
Could not stop docker containers with the commands above. The error message shows - "Permission Denied."

### Solution1 (x) 
#### Restart Docker service
`sudo service docker restart`

This may work for the moment, but needs to be done every time, and thus does not solve the problem.

### Solution2 (o)
#### Step 1.  Purge AppArmor

`sudo apt-get purge --auto-remove apparmor`: 

***Warning***: it would remove everything regarding apparmor, including applications installed over Snap. If you are not sure how would this affect your PC, then you may need to look up information before executing this, or refer to Solution3.

#### Step 2. Restart docker service
`sudo service docker restart`

#### Step 3. Remove all unused containers, networks, volumes, images, build cache.
`docker system prune -all --volumes`

#### Step 4. Reinstall docker-compose
`sudo apt-get install docker-compose`

#### Step 5. Reinstall AppArmor
`sudo apt-get install apparmor`

### Solution3 (o)

The only difference between this and solution2 is at Step1 & Step5.

#### Step 1. Disable AppArmor
An alternative of purging AppArmor, another way is to disable it.
```
sudo aa-status                                  # check status
sudo systemctl disable apparmor.service --now   # shutdown and prevent from restarting
sudo service apparmor teardown                  # unload AppArmor profiles
sudo aa-status                                  # check status again
```

#### Step 2. Restart docker service
`sudo service docker restart`

#### Step 3. Remove all unused containers, networks, volumes, images, build cache.
`docker system prune -all --volumes`

#### Step 4. Reinstall docker-compose
`sudo apt-install docker-compose`

#### Step 5. Reenable AppArmor
```
sudo systemctl enabled apparmor                 # enable AppArmor
sudo systemctl start apparmor                   # restart AppArmor
```

## Reference
* Stop containers
  - docker compose down - https://docs.docker.com/engine/reference/commandline/compose_down/
  - docker compose stop - https://docs.docker.com/engine/reference/commandline/compose_stop/
  - compare - https://stackoverflow.com/questions/46428420/docker-compose-up-down-stop-start-difference
  - compare - https://eldermoraes.com/docker-basics-how-to-start-and-stop-containers/
* Issue threads
  -  https://forums.docker.com/t/can-not-stop-docker-container-permission-denied-error/41142
  -  https://stackoverflow.com/questions/47223280/docker-containers-can-not-be-stopped-or-removed-permission-denied-error
* docker system prune - https://docs.docker.com/engine/reference/commandline/system_prune/
