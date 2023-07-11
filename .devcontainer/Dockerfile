FROM mcr.microsoft.com/devcontainers/base:jammy

RUN apt-get update && apt-get install -y \
  gcc-arm-none-eabi \
  bash-completion \
  && rm -rf /var/lib/apt/lists/*

USER vscode
