import argparse
import util

def nusmv_main():
    import nusmv
    parser = argparse.ArgumentParser()
    parser.add_argument('benchmark')
    parser.add_argument('-a', '--automata', default='automata')
    parser.add_argument('-f', '--function', default='verify')
    parser.add_argument('--args', nargs='+')
    args = parser.parse_args()
    module_path = util.model_filename(args.benchmark)
    automata = util.get_variable_from_module(module_path, args.automata)
    model = nusmv.Model(automata)
    if args.args is not None:
        arguments = [eval(arg) for arg in args.args]
    else:
        arguments = []
    print getattr(model, args.function)(*arguments)


def efsm_mc_main():
    import efsm_mc
    parser = argparse.ArgumentParser()
    parser.add_argument('benchmark')
    parser.add_argument('-a', '--automata', default='automata')
    parser.add_argument('-f', '--function', default='__str__')
    parser.add_argument('--args', nargs='+')
    args = parser.parse_args()
    module_path = util.model_filename(args.benchmark)
    automata = util.get_variable_from_module(module_path, args.automata)
    model = efsm_mc.Model(automata)
    if args.args is not None:
        arguments = [eval(arg) for arg in args.args]
    else:
        arguments = []
    print getattr(model, args.function)(*arguments)
