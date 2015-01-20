#!/bin/sh

ssh-keygen -t rsa -C "MinXu@github.com" -f ~/.ssh/id_rsa -N ""

#copy key....
xclip -sel clip < ~/.ssh/id_rsa.pub


#agent add
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_rsa

#show key.......
ssh-add -l


#testing......
ssh -T git@github.com
