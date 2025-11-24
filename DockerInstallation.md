# Docker setup for RoverFlake2

Minimal installation setup for Docker. Ask ChatGPT first if you are having trouble (do NOT run random commands unless you know exactly what they do). If you still can't figure it out, ask Aaron. Be sure to review the docs if you require any extra commands outside of what is listed.

### 1. Prerequisites
- Install Docker and Docker Compose on your machine
> docker --version
> docker compose version

### 2.1 Build the image (AMD64 Architecture):
> docker build -t rover2025:rover
### 2.2 Build the image (ARM64 Architecture):
> docker builx build --platform linux/amd64,linux/arm64 -t roverflake2:dev --push .

### 3. Start a detached container
> docker compose --compatibility up rover -d
(the compatibility flag might not work on Windows and if so, just remove it)

### 4. Go into an existing container
> docker compose exec rover bash 
(don't forget to change the service <rover> if you are using a different compose container)
