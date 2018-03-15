set nocompatible      " be iMproved, required
filetype on           " required
set number		      " show number
set wrap		      " don't show inside codes
" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim
syntax on
set encoding=utf-8
set autowrite
set cursorline
set autoindent
set cindent
set tabstop=4
set softtabstop=4
set shiftwidth=4
set ignorecase
set hlsearch
set incsearch
set showmatch

"Set mapleader
let mapleader = ","
"Fast reloading of the .vimrc
map <silent> <leader>ss :source ~/.vimrc<cr>
"Fast editing of .vimrc
map <silent> <leader>ee :e ~/.vimrc<cr>
"When .vimrc is edited, reload it
autocmd! bufwritepost .vimrc source ~/.vimrc 



call vundle#begin()
" alternatively, pass a path where Vundle should install plugins
"call vundle#begin('~/some/path/here')

" let Vundle manage Vundle, required
Plugin 'VundleVim/Vundle.vim'
Plugin 'scrooloose/nerdtree'
map <leader>t :NERDTreeToggle<CR>
map <F3> :NERDTreeToggle<CR>
let NERDTreeShowLineNumbers=1
let NERDTreeAutoCenter=1
let NERDTreeShowHidden=1
let NERDTreeWinSize=27
let g:nerdtree_tabs_open_on_console_startup=1
let NERDTreeIgnore=['\.pyc','\~$','\.swp']
let NERDTreeShowBookmarks=1
Plugin 'jistr/vim-nerdtree-tabs'
Plugin 'L9'
Plugin 'FuzzyFinder'
map <C-T> :FufFile<CR>
map <C-E> :MRU<CR>
" Specify the plugins you want to install here.
" We'll come on that later
" All of your Plugins must be added before the following line
call vundle#end()            " required
filetype plugin indent on    " required
" To ignore plugin indent changes, instead use:
"filetype plugin on
"
" Brief help
" :PluginList       - lists configured plugins
" :PluginInstall    - installs plugins; append `!` to update or just :PluginUpdate
" :PluginSearch foo - searches for foo; append `!` to refresh local cache
" :PluginClean      - confirms removal of unused plugins; append `!` to auto-approve removal
"
" see :h vundle for more details or wiki for FAQ
" Put your non-Plugin stuff after this line
" Put the rest of your .vimrc file here

