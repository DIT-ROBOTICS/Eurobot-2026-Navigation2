#!/bin/bash
# Bash completion script for nav2 command

_nav2_completions()
{
    local cur prev commands services
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    
    # Main commands
    commands="rebuild build run dev vnc stop logs exec ps tools clean install"
    
    # Service names
    services="build dev run vnc"
    
    # Complete main commands
    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=( $(compgen -W "${commands}" -- ${cur}) )
        return 0
    fi
    
    # Complete service names for specific commands
    if [ $COMP_CWORD -eq 2 ]; then
        case "${prev}" in
            stop|logs|exec)
                COMPREPLY=( $(compgen -W "${services}" -- ${cur}) )
                return 0
                ;;
        esac
    fi
}

complete -F _nav2_completions nav2
