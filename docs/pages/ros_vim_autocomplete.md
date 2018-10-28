# VIM autocomplete pour ROS

Installation (VIM autocomplete and VIM-Plug)

```bash
sudo apt-get install vim-nox-py2
curl -fLo ~/.vim/autoload/plug.vim --create-dirs https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
```

Editer VIMRC (.vimrc) et ajouter :

```bash
let g:ycm_semantic_triggers = {
			\   'roslaunch' : ['="', '$(', '/'],
			\   'rosmsg,rossrv,rosaction' : ['re!^', '/'],
			\ }

call plug#begin('~/.vim/plugged')

" VIM ROS
Plug 'taketwo/vim-ros'

" You complete me
Plug 'Valloric/YouCompleteMe'

" Initialize plugin system
call plug#end()
```
	
Ouvir VIM et entrer ":PluginsInstall"

Pour finaliser l'installation de *YouCompleteMe*, il faut lancer une compilation manuellement :

```bash
cd .vim/plugged/YouCompleteMe/
./install.py
```