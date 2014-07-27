"===============================================================================
"        Filename: vimrc
"          Author: min.xu
"           Email: xumin0903@126.com
"        Homepage: http://hi.baidu.com/
"         Created: 2010-08-02
"        Modified: 2014-04-21
"===============================================================================

"
" VIM

" When started as "evim", evim.vim will already have done these settings.
if v:progname =~? "evim"
  finish
endif

" Use Vim settings, rather then Vi settings (much better!).
" This must be first, because it changes other options as a side effect.
set nocompatible    "mgqw add: 取消vi兼容模式

set history=50        " keep 50 lines of command line history
set showcmd            " display incomplete commands

" For Win32 GUI: remove 't' flag from 'guioptions': no tearoff menu entries
" let &guioptions = substitute(&guioptions, "t", "", "g")

" Don't use Ex mode, use Q for formatting
map Q gq


" Only do this part when compiled with support for autocommands.
if has("autocmd")

  " Put these in an autocmd group, so that we can delete them easily.
  augroup vimrcEx

  " For all text files set 'textwidth' to 78 characters.
  autocmd FileType text setlocal textwidth=78

  " When editing a file, always jump to the last known cursor position.
  " Don't do it when the position is invalid or when inside an event handler
  " (happens when dropping a file on gvim).
  autocmd BufReadPost *
    \ if line("'\"") > 0 && line("'\"") <= line("$") |
    \   exe "normal! g`\"" |
    \ endif

  augroup END

endif " has("autocmd")

" Convenient command to see the difference between the current buffer and the
" file it was loaded from, thus the changes you made.
""command DiffOrig vert new | set bt=nofile | r # | 0d_ | diffthis
""			\| wincmd p | diffthis


"set fileencodings=default,ucs-bom,utf-8,cp936,gbk,gb2312,gb18030,latin1       "mgqw add: 设置读取文件编码的种类
set fileencodings=default,ucs-bom,utf-8,cp936,gb18030,latin1       "mgqw add: 设置读取文件编码的种类
"set encoding=UTF-8     "mgqw add: vim内部使用的字符编码方式,默认是什么就是什么,没有必要设置
"mgqw add: vim中当前编辑的文件的字符编码方式，vim保存文件时也会将文件保存为这种字符编码方式
"mgqw add: 没有必要强行改变文件的本身的编码方式, 屏蔽之
"set fileencoding=utf-8
set guifont=Luxi\ Mono\ 9   " 设置字体，字体名称和字号
"set tabstop=4      " 设置tab键的宽度(在后面函数SetCFileFormat()中进行设置)
"set expandtab        " 设置是否使用空格代替tab(在后面函数SetCFileFormat()中进行设置)
"set backspace=2    " 设置退格键可用
set backspace=indent,eol,start
set number!         " 显示行号
"set mouse=a            " 支持鼠标
"set visualbell     " vim进行编辑时，如果命令错误，会发出一个响声，该设置去掉响声
"set wrap           " 自动换行
"set nowrap         " 不自动换行
set linebreak       " 整词换行
set whichwrap=b,s,<,>,[,]       " 光标从行首和行末时可以跳到另一行去
"set list                       " 显示制表符
"set listchars = tab:>-,trail:- " 将制表符显示为'>---',将行尾空格显示为'-'
set listchars=tab:.\ ,trail:.   " 将制表符显示为'.   '
set autochdir                   " 自动设置目录为正在编辑的文件所在的目录
set hidden          " 没有保存的缓冲区可以自动被隐藏
set scrolloff=5

set cursorline        "mgqwadd: 设置高亮当前行
"--------------------------------------------------------------------------------
" 查找/替换相关的设置
"--------------------------------------------------------------------------------
set hlsearch        " 高亮显示搜索结果
set incsearch       " 查询时非常方便，如要查找book单词，当输入到/b时，会自动找到
                    " 第一个b开头的单词，当输入到/bo时，会自动找到第一个bo开头的
                    " 单词，依次类推，进行查找时，使用此设置会快速找到答案，当你
                    " 找要匹配的单词时，别忘记回车
set gdefault        " 替换时所有的行内匹配都被替换，而不是只有第一个
"nmap <F2> : nohlsearch <CR>"    "mgqw add:<F2>关闭高亮"

"--------------------------------------------------------------------------------
" 状态栏相关的设置
"--------------------------------------------------------------------------------
set statusline=[%F]%y%r%m%*%=[Line:%l/%L,Column:%c][%p%%]
set laststatus=2    " always show the status line 总是显示状态栏status line
set ruler           " 在编辑过程中，在右下角显示光标位置的状态行

"--------------------------------------------------------------------------------
" 编程相关的设置
"--------------------------------------------------------------------------------
set completeopt=longest,menu    " 关掉智能补全时的预览窗口
filetype plugin indent on       " 加了这句才可以用智能补全
syntax on              " 打开语法高亮
set showmatch       " 设置匹配模式，类似当输入一个左括号时会匹配相应的那个右括号
set smartindent     " 智能对齐方式
set tabstop=4
set shiftwidth=4    " 换行时行间交错使用4个空格
set autoindent      " 自动对齐
set ai              " 设置自动缩进
set cindent         " 设置C语言自动缩进
colorscheme desert  "mgqw add: 使用默认配色方案足够, 没必要改
set ignorecase      " 查找时忽略大小写

"--------------------------------------------------------------------------------
" 代码折叠
"--------------------------------------------------------------------------------
"set foldmarker={{{,}}}
set foldmethod=indent   " 设置折叠模式为利用自动缩进折叠, 这样不需要zf创建折叠
"set foldmethod=marker  " marker折叠方式需要用命令zf创建折叠才能用 zc/zo 折叠
"set foldmethod=syntax
"set nofoldenable
"set foldlevel=9         " Don't autofold anything (but I can still fold manually)
"set foldopen-=search   " don't open folds when you search into them
"set foldopen-=undo     " don't open folds when you undo stuff
"set foldcolumn=4
"autocmd BufWinLeave *.c,*.h mkview
"autocmd BufWinEnter *.c,*.h loadview
"zc 折叠
"zC 对所在范围内所有嵌套的折叠点进行折叠
"zo 展开折叠
"zO 对所在范围内所有嵌套的折叠点展开
"[z 到当前打开的折叠的开始处。
"]z 到当前打开的折叠的末尾处。
"zj 向下移动。到达下一个折叠的开始处。关闭的折叠也被计入。
"zk 向上移动到前一折叠的结束处。关闭的折叠也被计入。

" set mapleader
let mapleader=","       "mgqw注释: 设置map中的<leader>为 , 键

"--------------------------------------------------------------------------------
" 模仿MS Windows中的快捷键
"--------------------------------------------------------------------------------
"vmap <C-c> "yy
"vmap <C-x> "yd
"map <C-v> "yp
"vmap <C-v> "yp
"nmap <C-a> ggvG$

"--------------------------------------------------------------------------------
" 窗口操作的快捷键      mgqw add: 此快捷键会与单词移动快捷键w/e冲突,
                                " 造成移动速度很慢, 屏蔽
"--------------------------------------------------------------------------------
nmap sv     <C-w>v     " 垂直分割当前窗口
nmap sh     <C-w>s     " 水平分割当前窗口
"nmap wc     <C-w>c     " 关闭当前窗口

"Smart way to move btw. windows
"map <C-j> <C-W>j
"map <C-k> <C-W>k
"map <C-h> <C-W>h
"map <C-l> <C-W>l

"Tab configuration
map <leader>tn :tabnew %<cr>
map <leader>te :tabedit
map <leader>tc :tabclose<cr>
map <leader>tm :tabmove
try
  set switchbuf=usetab
  set stal=2
catch
endtry

"Fast saving
nmap <leader>w :w!<cr>         "mgqw add: 屏蔽此段代码


"Fast reloading of the .vimrc
"map <leader>s :source ~/.vimrc<cr>
"Fast editing of .vimrc
"map <leader>e :e! ~/.vimrc<cr>
"When .vimrc is edited, reload it   
"编译.vimrc的时候最好先屏蔽,不然老弹出错误提示很烦人
autocmd! bufwritepost .vimrc source ~/.vimrc


"###############################################################################
" The following is the Plugins' setting
"###############################################################################
"--------------------------------------------------------------------------------
" minibufexpl
"--------------------------------------------------------------------------------
let g:miniBufExplMapWindowNavVim = 1 " 按下Ctrl+h/j/k/l，可以切换到当前窗口的上下左右窗口
let g:miniBufExplMapWindowNavArrows = 1 " 按下Ctrl+箭头，可以切换到当前窗口的上下左右窗口
let g:miniBufExplMapCTabSwitchBufs = 1 " 启用以下两个功能：Ctrl+tab移到下一个buffer并在当前窗口打开；Ctrl+Shift+tab移到上一个buffer并在当前窗口打开；ubuntu好像不支持
let g:miniBufExplMapCTabSwitchWindows = 1 "启用以下两个功能：Ctrl+tab移到下一个窗口；Ctrl+Shift+tab移到上一个窗口；ubuntu好像不支持
let g:miniBufExplModSelTarget = 1  "不要在不可编辑内容的窗口（如TagList窗口）中打开选中的buffer
"--------------------------------------------------------------------------------
" TagList :Tlist
"--------------------------------------------------------------------------------
"let Tlist_Show_One_File=1  "mgqw add: 此设置会导致使用tabnew新打开文件后切换Tlist报错
let Tlist_Exit_OnlyWindow = 1
"let Tlist_Use_Right_Window=1
map <leader>t :Tlist<cr>

" Buffer
"map <leader>bn :bn<cr>
"map <leader>bp :bp<cr>
map <leader>q :close<cr>
map <leader>qa :qa<cr>
" Buffer switch by <leader>number
"map <leader>b1 :b1<cr>
"map <leader>b2 :b2<cr>
"map <leader>b3 :b3<cr>
"map <leader>b4 :b4<cr>
"map <leader>b5 :b5<cr>
"map <leader>b6 :b6<cr>
"map <leader>b7 :b7<cr>
"map <leader>b8 :b8<cr>
"map <leader>b9 :b9<cr>

"Use the arrows to something usefull
"map <right> :bn<cr>
"map <left> :bp<cr>

"For Cope
"map <silent> <leader><cr> :nohlsearch<cr>
"Paste toggle - when pasting something in, don't indent. 用鼠标复制粘贴时不自动缩进
set pastetoggle=<F3>    "用按F3键代替前面的设置语句


"--------------------------------------------------------------------------------
" netrw 文件浏览器 :e <PATH>
"--------------------------------------------------------------------------------
"let g:netrw_winsize = 30       " 浏览器宽度

"--------------------------------------------------------------------------------
" QuickFix 设置
"--------------------------------------------------------------------------------
"cscope搜索后quickfix键盘映射, 能够用F7和F8来切换QuickFix窗口中的项
"新建一个quickfix窗口显示cs find c的多个搜索结果
"leader+c 键盘对应:cw或者:copen
"nmap <leader>c :botright cw 10<cr>   
"leader+n 切换到下一个结果
"nmap <leader>n :cn<cr>               
"leader+p 切换到上一个结果
"nmap <leader>p :cp<cr>               
"F7 切换到下一个结果
"nmap <F7> :cn<cr>                   
"F8 切换到上一个结果
"nmap <F8> :cp<cr>                   
"leader+cc 关闭quickfix窗口
"nmap <leader>cc :cclose<cr>          

"--------------------------------------------------------------------------------
" WinManager :WMToggle
"--------------------------------------------------------------------------------
let g:winManagerWindowLayout='FileExplorer|TagList'
"let g:winManagerWidth = 30
"let g:defaultExplorer = 0
"nmap <C-w><C-b> :BottomExplorerWindow<cr> " 切换到最下面一个窗格
"nmap <C-w><C-f> :FirstExplorerWindow<cr>   " 切换到最上面一个窗格
"nmap wm :WMToggle<cr> " 是nomal模式的命令，不是Ex模式的


"--------------------------------------------------------------------------------
" cscope 在后面AddCscopeFuncList()函数中设置
"--------------------------------------------------------------------------------
"cs add /home/projects/cscope.out /home/projects
":set cscopequickfix=s-,c-,d-,i-,t-,e-
":set cscopetag
" 按下面这种组合键有技巧,按了<C-_>后要马上按下一个键,否则屏幕一闪
" 就回到nomal状态了
" <C-_>s的按法是先按"Ctrl+Shift+-",然后很快再按"s"
"nmap <C-_>s :cs find s <C-R>=expand("<cword>")<cr><cr> :cw<cr>
"nmap <C-_>g :cs find g <C-R>=expand("<cword>")<cr><cr> :cw<cr>
"nmap <C-_>c :cs find c <C-R>=expand("<cword>")<cr><cr> :cw<cr>
"nmap <C-_>t :cs find t <C-R>=expand("<cword>")<cr><cr> :cw<cr>
"nmap <C-_>e :cs find e <C-R>=expand("<cword>")<cr><cr> :cw<cr>
"nmap <C-_>f :cs find f <C-R>=expand("<cfile>")<cr><cr>
"nmap <C-_>i :cs find i <C-R>=expand("<cfile>")<cr><cr> :cw<cr>
"nmap <C-_>d :cs find d <C-R>=expand("<cword>")<cr><cr> :cw<cr>




"##################################################################
"########################## End Of Vimrc ##########################
"##################################################################

" if filetype is C
"   TODO
" fi




"mgqw add: 读取c或者cpp文件时打开Tlist附窗口, 并加载ctags
"autocmd FileType c,cpp set tags+=~/vimtag/sys-tags | set tags+=~/vimtag/apps-tags | set tags+=~/vimtag/tbsboot-tags

"autocmd FileType c,cpp set tags+=/home/laisimin/vimtag/kernel-tags




"--------------------------------------------------------------------------------
" mgqw add: 设置C/C++文件的格式
"--------------------------------------------------------------------------------
func! SetCFileFormat()
    "使用4个空格代替
    set tabstop=4       " 设置tab键的宽度
    set expandtab        " 设置是否使用空格代替tab
endfunc

"--------------------------------------------------------------------------------
" mgqw add: 定义tags文件路径局部变量函数
"--------------------------------------------------------------------------------
func! DefinePathParameters()
    " mgqw add: 设置vim的ctags,cscope插件数据文件的路径
    let s:ctagFileDir       = $HOME."/vimplug/data/"           
    " mgqw add: 设置ctags文件的路径
    let s:sys_tags          = s:ctagFileDir."sys-tags"
    let s:apps_tags         = s:ctagFileDir."apps-tags"
    let s:opensource_tags   = s:ctagFileDir."opensource-tags"
    let s:tbsboot_tags      = s:ctagFileDir."tbsboot-tags"
    let s:kernel_tags       = s:ctagFileDir."kernel-tags"
    " mgqw add: cscope数据文件的路径
    let s:cscope_data       = s:ctagFileDir."cscope.out"
    "echo "------------".s:.ctagFileDir"--------------"
endfunc

"--------------------------------------------------------------------------------
" mgqw add: 删除DefinePathParameters()函数定义的tags文件局部变量函数
"--------------------------------------------------------------------------------
func! DeletePathParameters()
    unlet s:ctagFileDir
    unlet s:sys_tags
    unlet s:apps_tags
    unlet s:opensource_tags
    unlet s:tbsboot_tags
    unlet s:kernel_tags
    unlet s:cscope_data
endfunc


"mgqw add: 只有c,cpp文件才会调用相应函数加载ctags和cscope配置
autocmd FileType c,cpp call AddCFileFuncList()
func! AddCFileFuncList()

    "代码文件格式设置函数
    call SetCFileFormat()

    "mgqw add: 初始化tags文件路径变量
    call DefinePathParameters()

    "mgqw add: 添加ctags设置
    call AddCtagsFuncList()
    "mgqw add: 添加cscope设置
    call AddCscopeFuncList()

    "mgqw add: 删除变量定义
    call DeletePathParameters()

endfunc


"--------------------------------------------------------------------------------
" mgqw add: 加载ctags设置属性函数定义
"--------------------------------------------------------------------------------
func! AddCtagsFuncList()

    "新的使用变量定义的相对路径的版本(两种设置语法随便二选一):
    "方法一的语法
    "let &tags .= ',' . s:sys_tags    
    
    "方法二的语法: 效果与方法一一样
    execute "set tags+=".s:sys_tags    
    execute "set tags+=".s:apps_tags
    "execute "set tags+=".s:opensource_tags
    "execute "set tags+=".s:tbsboot_tags
    "execute "set tags+=".s:kernel_tags

    "老的使用绝对路径的版本
    "set tags+=~/vimplug/data/sys-tags
    "set tags+=~/vimplug/data/apps-tags
    "set tags+=~/vimplug/data/opensource-tags
    "set tags+=~/vimplug/data/tbsboot-tags
    "set tags+=/home/laisimin/vimtag/kernel-tags
endfunc

"--------------------------------------------------------------------------------
" mgqw add: 加载cscope设置属性函数定义
"--------------------------------------------------------------------------------
func! AddCscopeFuncList()
    if has("cscope")
        " aUse both cscope and ctag
        set cscopetag
        " Use tags for definition search first
        set cscopetagorder=1
        " Use quickfix window to show cscope results
        set cscopequickfix=s-,g-,d-,c-,t-,e-,f-,i-
        "mgqw add: 不显示数据库是否添加成功,屏蔽错误输出"重复的 cscope 数据库未被加入(/etc/vimrc中已经载入一次)"
        set nocscopeverbose
        "mgqw add: 老的使用绝对路径添加cscope数据文件
        "if filereadable("/home/laisimin/vimplug/data/cscope.out")
            "cscope add   /home/laisimin/vimplug/data/cscope.out
        "endif
        "mgqw add: 新的的使用变量路径添加cscope数据文件
        if filereadable(s:cscope_data)
            execute "cscope add ".s:cscope_data
        endif
        " Show msg when cscope db added
        set cscopeverbose

        "mgqw add: 添加组合键映射
        " 按下面这种组合键有技巧,按了<C-\>后要马上按下一个键,否则屏幕一闪,就回到nomal状态了
        " <C-\>s的按法是先按"Ctrl+\",然后很快再按"s"
        nmap <C-\>s :cs find s <C-R>=expand("<cword>")<CR><CR>  
        nmap <C-\>g :cs find g <C-R>=expand("<cword>")<CR><CR>  
        nmap <C-\>c :cs find c <C-R>=expand("<cword>")<CR><CR>  
        nmap <C-\>t :cs find t <C-R>=expand("<cword>")<CR><CR>  
        nmap <C-\>e :cs find e <C-R>=expand("<cword>")<CR><CR>  
        nmap <C-\>f :cs find f <C-R>=expand("<cfile>")<CR><CR>  
        nmap <C-\>i :cs find i <C-R>=expand("<cfile>")<CR><CR>
        nmap <C-\>d :cs find d <C-R>=expand("<cword>")<CR><CR>  
    endif
endfunc






"-----------------------------下面的代码都是测试用--------------------------------

let mytype = &filetype
"echo "xxxxxxxxxxxxxxxxxxxxxxxxx=============mytpye  ["mytype"]========================================"
"echo "xxxxxxxxxxxxxxxxxxxxxxxxx=============filetype["&filetype"]========================================"
"echo "xxxxxxxxxxxxxxxxxxxxxxxxx=============ft      ["&ft"]========================================"

if &filetype=="c"
"if &ft == "c"
    "echo "-----------------------file type was c------------------------------"
    " 'filetype' is c
elseif &filetype == "cpp"
    "echo "-----------------------file type was cpp------------------------------"
    " it is cpp
else
    "echo "-----------------------file type was other------------------------------"
    " it is something else
endif 



"""""""""""""session viminvo"""""""""""
set sessionoptions-=curdir
set sessionoptions+=sesdir
autocmd VimLeave * mksession! $HOME/.vim/Session.vim
autocmd VimLeave * wviminfo! $HOME/.vim/viminfo
func! Restore_Session()
	source $HOME/.vim/Session.vim
	rviminfo $HOME/.vim/viminfo
endfunc

map <silent> rs :call Restore_Session() <CR><CR>

"""""""""""""设置开启ctags"""""""""""""   
set tags=$HOME/cmc/tags
set autochdir"
" 按下F12重新生成tag文件，并更新taglist
"map <F12> :!find $HOME/cmc/ -name "*.h" -or -name "*.hpp" -or -name "*.c" -or -name "*.cc" -or -name "*.cpp" > $HOME/cmc/cscope.file;cscope -bkq -i $HOME/cmc/cscope.file;ctags -R  --fields=+lS --c++-kinds=+p --fields=+iaS --extra=+q $HOME/cmc/ <CR><CR>:TlistUpdate<CR>

map <F12> :!cscope -Rbkq -f $HOME/cmc/cscope.out -s $HOME/cmc/;ctags --fields=+lS --c++-kinds=+p --fields=+iaS --extra=+q  --languages=c++ --langmap=c++:+.inl -h +.inl --c++-kinds=+px --fields=+aiKSz --extra=+q --exclude=lex.yy.cc --exclude=copy_lex.yy.cc -R -f $HOME/cmc/tags $HOME/cmc <CR><CR>:TlistUpdate<CR>

"""""""""""""""""Taglist设置"""""""""""""""""
let Tlist_Auto_Open = 0  
let Tlist_Ctags_Cmd = '/usr/bin/ctags' 
let Tlist_Show_One_File = 1 
let Tlist_Exit_OnlyWindow = 1
let Tlist_Inc_Winwidth=0
let Tlist_Use_Left_Window=1
let Tlist_File_Fold_Auto_Close=1
let Tlist_Process_File_Always=1
"nnoremap <silent> <F6> :TlistToggle<CR>

" Ctrl + T:  Switch on/off TagBa
nnoremap <silent> <c-l> :TagbarToggle<CR>
"let g:tagbar_left = 1                                "在左侧
let g:tagbar_right = 1                                "在右侧
let g:tagbar_width = 25                               "设置宽度
"autocmd VimEnter * nested :call tagbar#autoopen(1)    "打开vim时自动打开tagbar       

"""""""""""""""""winManager"""""""""""""""""""""""
let g:winManagerWindowLayout='FileExplorer|TagList'  

nmap wm :WMToggle<cr>  
nmap <silent> <F11> :WMToggle<cr>  

""""""""""""""""quickfix*************************
nmap <F5> :cw<cr>
nmap <F3> :cp<cr>
nmap <F4> :cn<cr>
""""""""""""""""""""""NERDTreeToggle设置"""""""""""""""""" 
"nnoremap <silent> <F7> :NERDTreeToggle<CR>
let NERDTreeIgnore = ['.*\.o$','.*\.ko$','.*\.gz$']
"""""""""""""""""BufExplorer设置"""""""""""""""""
nnoremap <silent> <F10> :BufExplorer<CR>
let g:bufExplorerDefaultHelp=1
let g:bufExplorerShowRelativePath=1
"""""""""""""""""Minbufexpl设置"""""""""""""""""
let g:miniBufExplMapWindowNavVim = 1
let g:miniBufExplMapWindowNavArrows = 1
let g:miniBufExplMapCTabSwitchBufs = 1
let g:miniBufExplModSelTarget = 1
let g:miniBufExplMinHeight = 1
let g:miniBufExplorerAutoUpdate = 1

let g:miniBufExplorerMoreThanOne = 2
let g:miniBufExplUseSingleClick = 1
"let g:miniBufExplVSplit = 25
"let g:miniBufExplSplitBelow=1

let g:bufExplorerSortBy = "name"

autocmd BufRead,BufNew :call UMiniBufExplorer

map <leader>u :TMiniBufExplorer<cr>
"""""""""""""""""cscope设置"""""""""""""""""
set cscopequickfix=s-,c-,d-,i-,t-,e-
if has("cscope")
    set csprg=/usr/local/bin/cscope
    set csto=1
    set cst
    set cspc=3
    set nocsverb
    if filereadable($HOME."/cmc/cscope.out")
	"""echo "add any database in current directory"
	cs add $HOME/cmc/cscope.out
    endif 
    set csverb
endif  

""""""""""""h/c""""""""""""""""""""""""""
nnoremap <silent> <C-H> :A<CR>
""""""""""""grep""""""""""""""""""""""""
nnoremap <silent> <F7> :Grep<CR>
""""""""""""自动补全"""""""""""""""""""
filetype plugin indent on
set completeopt=longest,menu
let g:SuperTabRetainCompletionType=2
let g:SuperTabDefaultCompletionType="<C-X><C-O>"

"""""""""""""""""omnicppcomplete setting""""""""""""""""""""""""""""""""""""
"" 按下F3自动补全代码，注意该映射语句后不能有其他字符，包括tab；否则按下F3会自动补全一些乱码
imap <F3> <C-X><C-O>
" 按下F2根据头文件内关键字补全
"imap <F2> <C-X><C-I>
set completeopt=menu,menuone " 关掉智能补全时的预览窗口
let g:OmniCpp_MayCompleteDot = 1 " autocomplete with .
let g:maniCpp_MayCompleteArrow = 1 " autocomplete with ->

let g:mniCpp_MayCompleteScope = 1 " autocomplete with ::
let g:mniCpp_SelectFirstItem = 1 " select first item (but don't insert)
let g:mniCpp_NamespaceSearch = 2 " search namespaces in this and included files
let g:mniCpp_ShowPrototypeInAbbr = 1 " show function prototype in popup window
let g:mniCpp_GlobalScopeSearch=1 " enable the global scope search
let g:mniCpp_DisplayMode=1 " Class scope completion mode: always show all members
let g:mniCpp_DefaultNamespaces=["std"]
let g:mniCpp_ShowScopeInAbbr=1 " show scope in abbreviation and remove the last column
let g:mniCpp_ShowAccess=1 


""""""""""""""""""""""""visualmark"""""""""""""""""""""""""""""""
map <c-F8> <Plug>Vm_toggle_sign
map mm <Plug>Vm_toggle_sign
map <F8> <Plug>Vm_goto_next_sign
map <s-F8> <Plug>Vm_goto_prev_sign

"""""""""""""""""""""""""""""echofunc""""""""""""""""""""""""""""""""""
""这个两个键可以通过设置g:EchoFuncKeyNext和g:EchoFuncKeyPrev参数来修改。这个插件需要tags文件的支持
"Alt+-    Alt+=
"ctags -R --fields=+lS
"括号匹配
"inoremap ( ()<ESC>i
"inoremap ) <c-r>=ClosePair(')')<CR>
"inoremap { {}<ESC>i
"inoremap } <c-r>=ClosePair('}')<CR>
"inoremap [ []<ESC>i
"inoremap ] <c-r>=ClosePair(']')<CR>
"inoremap < <><ESC>i
"inoremap > <c-r>=ClosePair('>')<CR>
let g:EchoFuncKeyNext='<F4>'
let g:EchoFuncKeyPrev='<F3>'
let g:EchoFuncAutoStartBalloonDeclaration = 1
let g:EchoFuncLangsUsed=["c","java","cpp"]
""""""""""""""""""""""""""tasklist""""""""""""""""""""""""""""""""""""
map <silent> <F6> <plug>TaskList

"""""""""""""""""""""""""下划线 行高亮""""""""""""""""""""""""""""""""
"'cterm'选项要设置,否则下划线仍然出现
"set cursorcolumn      " 设置高亮当前列
""hi CursorColumn cterm=underline ctermbg=blue ctermfg=NONE gui=underline  guibg=blue guifg=NONE  "设定高亮列的颜色为：
set cursorline        " 设置高亮当前行
hi CursorLine cterm=underline ctermbg=blue ctermfg=NONE gui=underline guibg=blue guifg=NONE       "设定高亮行的颜色为:

""""""""""搜索高亮取消"""""""""""""""""""""""""""""""""
map <F2> :nohlsearch <CR>

""""""""""powerline"""""""""""""""""""""""""""""""""""
"set guifont=PowerlineSymbols\ for\ Powerline 
"set nocompatible 
"set t_Co=256 
let g:Powerline_symbols = 'fancy' 

"""""""""doxygen""""""""""""""""""""""""""""""""""""""
let g:DoxygenToolkit_briefTag_pre="@Brief " 
let g:DoxygenToolkit_paramTag_pre="@Param " 
let g:DoxygenToolkit_returnTag="@Returns "
let g:DoxygenToolkit_authorTag="@author  "
let g:DoxygenToolkit_dateTag="@date "
let g:DoxygenToolkit_versionTag="@version "
let g:DoxygenToolkit_blockHeader="--------------------------------------------------------------------------" 
let g:DoxygenToolkit_blockFooter="----------------------------------------------------------------------------" 
let g:DoxygenToolkit_authorName="MinXu, min.xu@harman.com"
let g:DoxygenToolkit_licenseTag =                               "Project         Harman Car Multimedia System\<enter>"
let g:DoxygenToolkit_licenseTag = g:DoxygenToolkit_licenseTag . "(C) copyright   2014\<enter>"
let g:DoxygenToolkit_licenseTag	= g:DoxygenToolkit_licenseTag . "Company         Harman/Becker Automotive Systems GmbH\<enter>"
let g:DoxygenToolkit_licenseTag	= g:DoxygenToolkit_licenseTag . "                All rights reserved\<enter>"
let g:DoxygenToolkit_licenseTag = g:DoxygenToolkit_licenseTag . "Secrecy Level   STRICTLY CONFIDENTIAL"

let g:DoxygenToolkit_briefTag_funcName="yes"
let g:DoxygenToolkit_compactOneLineDoc = "yes"
let g:DoxygenToolkit_commentType = "C" 
let g:doxygen_enhanced_color=1

""""""""""""""""YouCompleteMe""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:ycm_global_ycm_extra_conf = $HOME."/.vim/here/YouCompleteMe/cpp/ycm/.ycm_extra_conf.py"

""let g:clang_user_options='|| exit 0'
nnoremap <leader>jd :YcmCompleter GoToDefinitionElseDeclaration<CR>

" 补全功能在注释中同样有效
let g:ycm_complete_in_comments=1
" 允许 vim 加载 .ycm_extra_conf.py 文件，不再提示
let g:ycm_confirm_extra_conf=0
" 开启 YCM 基于标签引擎
let g:ycm_collect_identifiers_from_tags_files=1
" 引入 C++标准库tags，这个没有也没关系，只要.ycm_extra_conf.py文件中指定了正确的标准库路径
""set tags+=/data/misc/software/misc./vim/stdcpp.tags
" YCM 集成 OmniCppComplete 补全引擎，设置其快捷键
""inoremap <leader>; <C-x><C-o>
" 补全内容不以分割子窗口形式出现，只显示补全列表
set completeopt-=preview
" 从第一个键入字符就开始罗列匹配项
let g:ycm_min_num_of_chars_for_completion=1
" 禁止缓存匹配项，每次都重新生成匹配项
let g:ycm_cache_omnifunc=0
" 语法关键字补全
let g:ycm_seed_identifiers_with_syntax=1
" 修改对C函数的补全快捷键，默认是CTRL + space，修改为ALT + ;
let g:ycm_key_invoke_completion = '<M-;>'
" 设置转到定义处的快捷键为ALT + G，这个功能非常赞
nmap <M-g> :YcmCompleter GoToDefinitionElseDeclaration<C-R>=expand("<cword>")<CR><CR>


""""""""""""""""""clang_complete"""""""""""""""""""""""""""""""""""""""""""""
""autocmd VimEnter nested * !cp $HOME/.vim/here/clang_complete/plugin/.clang_complete $HOME/cmc/
autocmd VimLeave * !rm $HOME/cmc/.clang_complete
silent! execute '!cp -f $HOME/.vim/here/clang_complete/plugin/.clang_complete $HOME/cmc/'

let g:clang_complete_auto=1
let g:clang_complete_copen=1
let g:clang_periodic_quickfix=0
let g:clang_snippets=1
let g:clang_close_preview=1
let g:clang_use_library=1
let g:clang_user_options='-stdlib=libc++ -std=c++11 -IIncludePath'
let g:clang_complete_macros=1
let g:clang_hl_errors=1
let g:clang_complete_patterns=1
let g:clang_library_path='/usr/lib'
let g:clang_auto_user_options='path, .clang_complete'
let g:neocomplcache_enable_at_startup = 1



""""""""""""""""""syntastic""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:syntastic_check_on_open = 1
let g:syntastic_cpp_include_dirs = ['/usr/include/']
let g:syntastic_cpp_remove_include_errors = 1
let g:syntastic_cpp_check_header = 1
let g:syntastic_cpp_compiler = 'clang++'
let g:syntastic_cpp_compiler_options = '-std=c++11 -stdlib=libstdc++'
"set error or warning signs
let g:syntastic_error_symbol = '✗'
let g:syntastic_warning_symbol = '⚠'
""whether to show balloons
let g:syntastic_enable_balloons = 1

""""""""""""""""""highlight Functions"""""""""""""""""""""""""""""""""""""""""""""""""""
func! HighlightFunctionsAndClasses()
	syn match cFunctions "\<[a-zA-Z_][a-zA-Z_0-9]*\>[^()]*)("me=e-2
	syn match cFunctions "\<[a-zA-Z_][a-zA-Z_0-9]*\>\s*("me=e-1
	syn match cClass "\<[a-zA-Z_][a-zA-Z_0-9]*\>::"me=e-2
	hi cFunctions guifg=#7fd02e cterm=bold ctermfg=Magenta
	hi cClass guifg=#7fd02e cterm=bold ctermfg=Magenta
endfunc
au Syntax * call HighlightFunctionsAndClasses()

""""""""""""""""""CTRLP"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
let g:ctrlp_map = '<c-p>'

""""""""""""""""""comments""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
""x,cc  x,line number comments
""x,cu                uncomments
""x,cs                /**/ comments


"""""""""""""""""""GVIM""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
""helptags ~/.vim/doc
""help vimgdb
""run macros/gdb_mappings.vim""""<F7>
""""""space ---->command-T windows open
""""""gdb   ---->command-T windows close
""cd ~/cmc
""file aa   ---->load debug binary
""bel 20vsplit gdb-variables          ---->open the virialble windows
"""CTRL + P       添加变量 v先选中    ----->添加监控变量
"""createvar *r                       ----->添加监控局部变量
""""CTRL + X                          ----->打印光标下指针指向的变量值
"""CTRL + B       ------>添加断点
"""CTRL + E       ------>清除光标行断点
"""R              运行
"""CTRL + N       继续
"""C              继续
"""S              单步运行
func! GdbSetting()
	set previewheight=12 " set gdb window initial height
	set gdbprg=/usr/bin/gdb
	run macros/gdb_mappings.vim     " source key mappings listed in this document
<F7>
	call ToggleGDB()
endfunc
map <silent><c-g> :call GdbSetting()<CR><CR>

""""""""""""""""""""nerdcommenter""""""""""""""""""""""""""""""
""3,cc   注释
""3,cu   取消注释
""3,cs   块注释


""""""""""""""""""""indentline"""""""""""""""""""""""""""""""""
let g:indentLine_color_term = 239
let g:indentLine_color_gui = '#A4E57E' 
let g:indentLine_color_tty_light = 7
let g:indentLine_color_dark = 1
let g:indentLine_char = '│'


""""""""""""""""""""mru""""""""""""""""""""""""""""""""""""""""
let MRU_File = $HOME."/.vim/.vim_mru_files"
""设置默认保存最近打开文件列表的文件，默认$HOME.vim_mru_files；
let MRU_Max_Entries = 1000 
""设置默认记录的打开文件数量，默认100；
let MRU_Exclude_Files = '^/tmp/.*|^/var/tmp/.*'
""设置默认不包含哪些文件，默认空；
let MRU_Include_Files = '\.c$\|\.h$\|\.cpp$\|\.hpp$' 
""设置默认只包含哪些文件，默认空；
let MRU_Window_Height = 15 
""设置MRU窗口的高度，默认8；
let MRU_Use_Current_Window = 1
""设置默认在本窗口打开最近文件列表，而不是新的窗口；
let MRU_Auto_Close = 0 
""设置默认不关闭MRU窗口，默认关闭；
let MRU_Max_Menu_Entries = 20 
""设置MRU窗口默认显示多少条记录，默认10；
let MRU_Max_Submenu_Entries = 15
""设置MRU窗口默认子菜单显示多少条记录，默认10。如果有太多的记录显示在MRU窗口时，记录会被分割成子菜单，该配置设置子菜单里显示多少条记录；

""""""""""""""""""""lookupfile setting"""""""""""""""""""""""""
let g:LookupFile_MinPatLength = 2               "最少输入2个字符才开始查找
let g:LookupFile_PreserveLastPattern = 0        "不保存上次查找的字符串
let g:LookupFile_PreservePatternHistory = 1     "保存查找历史
let g:LookupFile_AlwaysAcceptFirst = 1          "回车打开第一个匹配项目
let g:LookupFile_AllowNewFiles = 0              "不允许创建不存在的文件
if filereadable($HOME."/cmc/tags")                "设置tag文件的名字
	let g:LookupFile_TagExpr = $HOME."/cmc/tags"
endif
let g:LookupFile_DisableDefaultMap = 1
"映射LookupFile为,lk
nmap <silent> <leader>lk :LUTags<cr>
""映射LUBufs为,ll
nmap <silent> <leader>ll :LUBufs<cr>
"映射LUWalk为,lw
nmap <silent> <leader>lw :LUWalk<cr>
""""""""""""""""""""pathogen""""""""""""""""""""""""""""""""""""""""""""
if v:version > 703 || (v:version == 703 && has('patch584'))
	source $HOME/.vim/here/vim-pathogen/autoload/pathogen.vim
	call pathogen#infect('$HOME/.vim/here/{}') 
	call pathogen#helptags()
endif

""""""""""""""""""""vunpath""""""""""""""""""""""""""""""""""""""""""""
" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/here/vundle/
call vundle#rc()
" alternatively, pass a path where Vundle should install plugins
let vunpath = '~/.vim/here'
call vundle#rc(vunpath)

Plugin 'gmarik/vundle'
Plugin 'Lokaltog/vim-powerline.git'
Plugin 'tpope/vim-fugitive'
Plugin 'Lokaltog/vim-easymotion'
Plugin 'tpope/vim-rails.git'

Plugin 'L9'
Plugin 'FuzzyFinder'

Plugin 'tasklist.vim'
Plugin 'taglist.vim'
Plugin 'Visual-Mark'
""Plugin 'OmniCppComplete'
Plugin 'Rip-Rip/clang_complete'
if v:version > 703 || (v:version == 703 && has('patch584'))
	Plugin 'Valloric/YouCompleteMe'
endif
Plugin 'grep.vim'
Plugin 'echofunc.vim'
Plugin 'a.vim'
Plugin 'c.vim'
Plugin 'corntrace/bufexplorer'
Plugin 'scrooloose/nerdcommenter'
Plugin 'scrooloose/nerdtree'
Plugin 'scrooloose/syntastic'

""Plugin 'fholgado/minibufexpl.vim'
Plugin 'SuperTab'
Plugin 'tagbar-phpctags'
Plugin 'winmanager'
Plugin 'cscope.vim'
Plugin 'wolfpython/cscope_map.vim'
Plugin 'DoxygenToolkit.vim'
""Plugin 'project.vim'
Plugin 'tpope/vim-pathogen'
Plugin 'kien/ctrlp.vim'
Plugin 'vim-scripts/AutoClose'
Plugin 'majutsushi/tagbar'
Plugin 'bronson/vim-trailing-whitespace'
Plugin 'Yggdroot/indentLine'
Plugin 'mru.vim'
Plugin 'vimgdb_runtime'
Plugin 'lookupfile'
Plugin 'genutils'

""gvim"""""""""""""""""""""function gvim () { (/usr/bin/gvim -f "$@" &) }""""""""""
"""""""""""""""""""""""""""function gvim () { (/usr/bin/gvim -f "$@" &) }""""""""""
""""""fix the bug : gvim -f ~/cmc/aa.c"""""""""""""""""""""""""""""""""""""""""""""


""vim-addons"""""""""""vim-addons install omnicppcomplete""""""""""""""""""""""""""
"""""""""""""""""""""""sudo apt-get install vim vim-scripts vim-doc""""""""""""""""
"""""""""""""""""""""""sudo apt-get install vim-addon-manager""""""""""""""""""""""

""Bundle"""""""""""""""git clone https://github.com/gmarik/vundle.git  ~/.vim/bundle/vundle""""""

""syntastic""""""""""""sudo apt-get install splint"""""""""""""""""""""""""""""""""

""install VIM""""""""""sudo apt-get install python-dev"""""""""""""""""""""""""""""
"""""""""""""""""""""""sudo add-apt-repository ppa:nmi/vim-snapshots"""""""""""""""
"""""""""""""""""""""""sudo apt-get update"""""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""sudo apt-get install vim""""""""""""""""""""""""""""""""""""

""YCM"""""""""""""""""""""./install.sh--clang-completer""""""""""""""""""""""""""""

""clang complete"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""make install""""""""""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""sudo apt-get install clang-3.3-dev""""""""""""""""""""""""
"""""""""""""""""""""""""dpkg -l libclang.so& cp xxx/libclang.so /usr/lib/"""""""""
"""""""""""""""""""""""""cp .clang_complete ~/"""""""""""""""""""""""""""""""""""""

""vimgdb"""""""""""""""""""""https://github.com/larrupingpig/vimgdb-for-vim7.3"""""
"""""""""""""""""""""""""patch -p0 <vimgdb-for-vim7.3/vim73.patch""""""""""""""""""
"""""""""""""""""""""""""cp-rf/tmp/vimgdb-for-vim7.3/vimgdb_runtime/*~/.vim""""""""
"""""""""""""""""""""""""Makefile uncomment prefix =$(HOME)""""""""""""""""""""""""
"""""""""""""""""""""""""Makefile comment BINDIR   = /usr/bin""""""""""""""""""""""
"""""""""""""""""""""""""sudo apt-get install libncurses5-dev""""""""""""""""""""""
"""""""""""""""""""""""""sudo apt-get install libx11-dev libxt-dev libgtk2.0-dev"""

""git"""""""""""""""""""""""git init"""""""""""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""git submodule add插件的Git仓库地址bundle/插件名字"""""""""
"""""""""""""""""""""""""git checkout master; git pull"""""""""""""""""""""""""""""
"""""""""""""""""""""""""git submodule foreach 'git checkout master && git pull'"""
"""""""""""""""""""""""""git rm -r bundle/插件名"""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""git status""""""""""""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""git add .vimrc""""""""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""git commit -m 'ADD: pathogen & nerdtree'""""""""""""""""""
"""""""""""""""""""""""""git remote add origin git@github.com:perfectworks/vim.git"
"""""""""""""""""""""""""git push origin master""""""""""""""""""""""""""""""""""""


""git configure""""""""""""""""git config --global user.name "Your Name Here"
"""""""""""""""""""""""""""""""# Sets the default name for git to use when you commit
"""""""""""""""""""""""""""""""git config --global user.email "your_email@example.com"
"""""""""""""""""""""""""""""""# Sets the default email for git to use when you commit
"""""""""""""""""""""""""""""""git config --global credential.helper cache
"""""""""""""""""""""""""""""""# Set git to use the credential memory cache
"""""""""""""""""""""""""""""""git config --global credential.helper 'cache --timeout=3600'
"""""""""""""""""""""""""""""""# Set the cache to timeout after 1 hour (setting is in seconds)

