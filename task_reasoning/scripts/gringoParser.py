#! /usr/bin/env python
#
#  Authors: Francesco Trapani,
#  Benjamin Andres, Philipp Obermeier, Orkunt Sabuncu, Torsten Schaub, David Rajaratnam
#
# Utility class for converting strings into ASP assertions
import clingo


def string2fun(string):
    """
    Converts a string into an ASP assertion
    """
    parenthesis = string.find("(")
    if string.isdigit():
        return int(string)
    elif parenthesis == -1:
        return clingo.Function(string, [])
    elif parenthesis == 0:
        fun_tuple = ()
        for element in _string2list(string[1:-1]):
            fun_tuple = fun_tuple + (string2fun(element),)
        return fun_tuple
    else:
        arguments = list(map(string2fun, _string2list(string[parenthesis + 1:-1])))
        for idx in range(len(arguments)):
            if type(arguments[idx]) == str:
                arguments[idx] = clingo.String(arguments[idx])
            elif type(arguments[idx]) == int:
                arguments[idx] = clingo.Number(arguments[idx])
        return clingo.Function(string[0:parenthesis], arguments)


def _string2list(string):
    """
    Converts a string representing a list of parameters into a list of strings
    """
    list = []
    open = 1
    last = 0
    for index in range(len(string)):
        if string[index] == "(":
            open += 1
        elif string[index] == ")":
            open -= 1
        elif string[index] == "," and open == 1:
            list.append(string[last:index])
            last = index + 1
    list.append(string[last:])
    return list
