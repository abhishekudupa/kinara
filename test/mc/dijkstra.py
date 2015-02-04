import itertools
import pyparsing as p
import networkx as nx
import pystache
import subprocess
import sys

def guard1_middle_machine(xL, xS, xR, upS, upR):
    return (xS != xL)


def guard2_middle_machine(xL, xS, xR, upS, upR):
    return (xS == xR and upS and (not upR))


def update1_data_middle_machine(xS, upS):
    return not xS


def update2_data_middle_machine(xS, upS):
    return xS


def update1_up_middle_machine(xS, upS):
    return True


def update2_up_middle_machine(xS, upS):
    return False


def guard_0_machine(xS, xR, upR):
    return (xS == xR) and (not upR)


def update_data_0_machine(xS, xR, upR):
    return not xS


def guard_N_machine(xL, xS):
    return xL != xS


def update_data_N_machine(xL, xS):
    return not xS


def states(number_of_nodes):
    assert number_of_nodes >= 3
    # each node has an x and an up, except for the last and the first,
    # which don't have an up
    # a state is a tuple of tuples
    choices = []
    for i in range(number_of_nodes):
        if i == 0:
            choices.append([True, False])
            choices.append([True])
        elif i == number_of_nodes - 1:
            choices.append([True, False])
            choices.append([False])
        else:
            choices.append([True, False])
            choices.append([True, False])
    return list(itertools.product(*choices))


def get_input_params_for_process(state, process_id, number_of_processes):
    if process_id == 0:
        # for process 0, it's xS, xR, and upR
        return (state[0], state[2], state[3])
    elif process_id == number_of_processes - 1:
        # for process N, it's xL, xS
        return (state[-4], state[-2])
    else:
        # for middle processes it's (xL, xS, xR, upS, upR)
        state_index = 2 * process_id
        return (state[state_index - 2], state[state_index], state[state_index + 2],
                state[state_index + 1], state[state_index + 3])


def make_graph():
    # go over all the states
    # add a note for each state


    # for each state, evaluate the guards of all the processes
    # check that only one guard is satisfied.
    # compute next state
    # add an edge
    pass


def parse_result_to_table(result_string):
    result_table = dict()
    line = p.Group(p.OneOrMore(p.Word(p.alphas)) + p.Literal("->") + p.Word(p.alphas))
    table = p.OneOrMore(line)
    parse_result = table.parseString(result_string)
    for line in parse_result:
        inputs = [(True if v == 'true' else False) for v in line[:5]]
        result = True if line[-1] else False
        result_table[tuple(inputs)] = result
    return result_table


guard_domain_length = 6


def produce_golden_model_file(output='golden_model.txt'):
    f = open(output, 'w')
    guard_domain = (guard_domain_length * [[False, True]])
    guard_valuations = list(itertools.product(*guard_domain))
    update_domain = (2 * [[False, True]])
    update_valuations = list(itertools.product(*update_domain))

    f.write('(Guard1 ...) -> {\n')
    for valuation in guard_valuations:
        result = guard1_middle_machine(*valuation)
        valuation = [bool_to_string(b) for b in valuation]
        input_vector = ' '.join(valuation[:guard_domain_length])
        f.write("{} -> {}\n".format(input_vector, bool_to_string(result)))
    f.write('}\n')

    f.write('(Guard2 ...) -> {\n')
    for valuation in guard_valuations:
        result = guard2_middle_machine(*valuation)
        valuation = [bool_to_string(b) for b in valuation]
        input_vector = ' '.join(valuation[:guard_domain_length])
        f.write("{} -> {}\n".format(input_vector, bool_to_string(result)))
    f.write('}\n')

    f.write('(Update1-Di ...) -> {\n')
    for valuation in update_valuations:
        result = update1_data_middle_machine(*valuation)
        valuation = [bool_to_string(b) for b in valuation]
        input_vector = ' '.join(valuation[:guard_domain_length])
        f.write("{} -> {}\n".format(input_vector, bool_to_string(result)))
    f.write('}\n')

    f.write('(Update2-Di ...) -> {\n')
    for valuation in update_valuations:
        result = update2_data_middle_machine(*valuation)
        valuation = [bool_to_string(b) for b in valuation]
        input_vector = ' '.join(valuation[:guard_domain_length])
        f.write("{} -> {}\n".format(input_vector, bool_to_string(result)))
    f.write('}\n')

    f.write('(Update1-Ui ...) -> {\n')
    for valuation in update_valuations:
        result = update1_up_middle_machine(*valuation)
        valuation = [bool_to_string(b) for b in valuation]
        input_vector = ' '.join(valuation[:guard_domain_length])
        f.write("{} -> {}\n".format(input_vector, bool_to_string(result)))
    f.write('}\n')

    f.write('(Update2-Ui ...) -> {\n')
    for valuation in update_valuations:
        result = update2_up_middle_machine(*valuation)
        valuation = [bool_to_string(b) for b in valuation]
        input_vector = ' '.join(valuation[:guard_domain_length])
        f.write("{} -> {}\n".format(input_vector, bool_to_string(result)))
    f.write('}\n')


def main():
    if len(sys.argv) < 3:
        print "python dijkstra.py NumProcesses ModelFile [OutputFileName]"
        sys.exit(1)

    number_of_processes = int(sys.argv[1])
    model_filename = sys.argv[2]

    output_filename = sys.argv[3] if len(sys.argv) == 4 else "Dijkstra4CheckProcesses.cpp"

    debug = "debug" in sys.argv

    print "Number of processes is {}".format(number_of_processes)

    print "Output Filename is {}".format(output_filename)

    table = parse_file(model_filename)

    render_check_process_template(table, number_of_processes, output_filename=output_filename)

    binary_filename = output_filename.rstrip(".cpp")
    command = ["make", "-f", "Makefile.mac",
               "PROJECT_EXECUTABLES={}".format(binary_filename),
               "-j8", "debug" if debug else "eopt"]
    print "Command is {}".format(' '.join(command))
    subprocess.call(command)
    subprocess.call(["bin/{}/{}".format("debug" if debug else "opt", binary_filename)])

def render_check_process_template(table, number_of_processes, output_filename="Dijkstra4CheckProcesses.cpp"):
    context = {}

    context['DataDefinitions'] = data_variable_definitions(number_of_processes)
    context['UpdateDefinitions'] = up_variable_definitions(number_of_processes)

    context['NumProcesses'] = number_of_processes
    # print context['DataDefinitions']
    # print context['UpdateDefinitions']

    number_of_middle_processes = number_of_processes - 2

    guard1_ites = [dictionary_to_ite('Guard1', table['Guard1'], process_id)
                   for process_id in range(number_of_middle_processes)]
    guard2_ites = [dictionary_to_ite('Guard2', table['Guard2'], process_id)
                   for process_id in range(number_of_middle_processes)]

    update_data_1_ites = [dictionary_to_ite('Update1-Di', table['Update1-Di'], process_id)
                   for process_id in range(number_of_middle_processes)]
    update_data_2_ites = [dictionary_to_ite('Update2-Di', table['Update2-Di'], process_id)
                   for process_id in range(number_of_middle_processes)]

    update_up_1_ites = [dictionary_to_ite('Update1-Ui', table['Update1-Ui'], process_id)
                   for process_id in range(number_of_middle_processes)]
    update_up_2_ites = [dictionary_to_ite('Update2-Ui', table['Update2-Ui'], process_id)
                   for process_id in range(number_of_middle_processes)]

    context['Guard1s'] = '{{ {} }};'.format(','.join(guard1_ites))
    context['Guard2s'] = '{{ {} }};'.format(','.join(guard2_ites))

    context['UpdateData1s'] = '{{ {} }};'.format(','.join(update_data_1_ites))
    context['UpdateData2s'] = '{{ {} }};'.format(','.join(update_data_2_ites))

    context['UpdateUp1s'] = '{{ {} }};'.format(','.join(update_up_1_ites))
    context['UpdateUp2s'] = '{{ {} }};'.format(','.join(update_up_2_ites))

    template_file_contents = open('Dijkstra4CheckProcessesTemplate.cpp', 'r').read()
    r = pystache.Renderer(escape=lambda u: u)

    output = open(output_filename, 'w')
    output.write(r.render(template_file_contents, context, escape=lambda u: u))


def parse_file(filename):
    f = open(filename, 'r')
    s = f.read()

    line = p.Group(p.OneOrMore(p.Word(p.alphas)) + p.Literal("->") + p.Word(p.alphas))
    table = p.Group(p.OneOrMore(line).setResultsName("Table"))

    # result = p.OneOrMore(p.Suppress(skip_to_next_table) + table + p.Literal("}")).parseString(s)
    # print result

    skip_to_next_table = (p.Literal("Model") | p.Literal("Transition")) + p.SkipTo("-> {", include=True)

    line = p.Group(p.OneOrMore(p.Word(p.alphas)) + p.Literal("->") + p.Word(p.alphas))

    table = p.Group(p.Suppress(p.SkipTo(p.Literal("("), include=True)) +
                    (p.Literal("Guard1") | p.Literal("Guard2") | p.Literal("Update1-Di") | p.Literal("Update1-Ui") | p.Literal("Update2-Di") | p.Literal("Update2-Ui")) +
                    # p.Word(p.alphas + p.nums) +
                    p.Suppress(p.SkipTo("-> {", include=True)) +
                    p.Group(p.OneOrMore(line)))

    result = p.OneOrMore(table + p.Suppress(p.Literal("}"))).parseString(s)
    table_line_to_tuple = lambda line: (tuple([to_bool(e) for e in line[:-2]]), to_bool(line[-1]))
    list_of_tuples = [(e[0], dict([table_line_to_tuple(l) for l in e[1]])) for e in result.asList()]
    results = dict(list_of_tuples)
    new_results = {}
    for name in results:
        table = results[name]
        if name.startswith('Guard'):
            table = dict([(k,v) for k, v in table.items() if v])
        new_results[name] = table
    results = new_results
    return results


def data_variable_definitions(number_of_processes):
    return '\n'.join(["auto D{}Exp = TheLTS->MakeVar(\"Data{}\", TheLTS->MakeBoolType());".format(i, i)
                      for i in range(number_of_processes)])


def up_variable_definitions(number_of_processes):
    return '\n'.join(["auto U{}Exp = TheLTS->MakeVar(\"Up{}\", TheLTS->MakeBoolType());".format(i, i)
                      for i in range(number_of_processes)])



def to_bool(s):
    """ s is either 'true' or 'false' """
    return (True if s == 'true' else False)


def bool_to_string(b):
    return 'true' if b else 'false'


def dictionary_to_ite(function_name, table, process_id):
    if function_name.startswith('Guard'):
        if guard_domain_length == 6:
            input_variables = ['D{}Exp'.format(process_id),
                               'D{}Exp'.format(process_id + 1),
                               'D{}Exp'.format(process_id + 2),
                               'U{}Exp'.format(process_id),
                               'U{}Exp'.format(process_id + 1),
                               'U{}Exp'.format(process_id + 2)]
        else:
            input_variables = ['D{}Exp'.format(process_id),
                               'D{}Exp'.format(process_id + 1),
                               'D{}Exp'.format(process_id + 2),
                               'U{}Exp'.format(process_id + 1),
                               'U{}Exp'.format(process_id + 2)]

    else:
        input_variables = ['D{}Exp'.format(process_id + 1),
                           'U{}Exp'.format(process_id + 1)]
    ite_lines = []
    for input_values, result_value in table.items():
        input_efsm_values = ['TheLTS->MakeTrue()' if val else 'TheLTS->MakeFalse()'
                             for val in input_values]
        input_conjuncts = ["TheLTS->MakeOp(LTSOps::OpEQ, {}, {})".format(input_variable, input_value)
                           for input_variable, input_value in zip(input_variables, input_efsm_values)]
        ite_lines.append(('TheLTS->MakeOp(LTSOps::OpAND, {{ {} }})'.format(', '.join(input_conjuncts)), 'TheLTS->MakeTrue()' if result_value else 'TheLTS->MakeFalse()'))
        # { })
        # print "TheLTS->Make"
    ite_expression = 'TheLTS->MakeFalse()'
    for guard, result in reversed(ite_lines):
        ite_expression = 'TheLTS->MakeOp(LTSOps::OpITE, {guard}, \n{result}, {old_ite})'.format(guard=guard, result=result, old_ite=ite_expression)
    return ite_expression



if __name__ == '__main__':
    main()


def draw_model():
    # G = nx.DiGraph()
    # for state in states(number_of_processes):
    #     G.add_node(state)

    # legitimate_states = []
    # for state in states(number_of_processes):
    #     print state
    #     num_enabled = 0
    #     for i in range(number_of_processes):
    #         inputs = get_input_params_for_process(state, i, number_of_processes)

    #         if i == 0:
    #             print inputs
    #             print guard_0_machine(*inputs)
    #             if guard_0_machine(*inputs):
    #                 num_enabled += 1
    #                 new_data = update_data_0_machine(*inputs)
    #                 new_state_list = list(state)
    #                 new_state_list[0] = new_data
    #                 new_state = tuple(new_state_list)
    #                 print "adding edge from {} to {}".format(state, new_state)
    #                 G.add_edge(state, new_state)

    #         elif i == number_of_processes - 1:
    #             print inputs
    #             print guard_N_machine(*inputs)
    #             if guard_N_machine(*inputs):
    #                 num_enabled += 1
    #                 new_data = update_data_N_machine(*inputs)
    #                 new_state_list = list(state)
    #                 new_state_list[-2] = new_data
    #                 new_state = tuple(new_state_list)
    #                 print "adding edge from {} to {}".format(state, new_state)
    #                 G.add_edge(state, new_state)

    #         else:
    #             print inputs
    #             guard1 = guard1_middle_machine(*inputs)
    #             guard2 = guard2_middle_machine(*inputs)
    #             if guard1:
    #                 num_enabled += 1
    #                 new_data = update1_data_middle_machine(*inputs)
    #                 new_up = update1_up_middle_machine(*inputs)
    #                 new_state_list = list(state)
    #                 new_state_list[2 * i] = new_data
    #                 new_state_list[2 * i + 1] = new_up
    #                 new_state = tuple(new_state_list)
    #                 print "adding edge from {} to {}".format(state, new_state)
    #                 G.add_edge(state, new_state)
    #             if guard2:
    #                 num_enabled += 1
    #                 new_data = update2_data_middle_machine(*inputs)
    #                 new_up = update2_up_middle_machine(*inputs)
    #                 new_state_list = list(state)
    #                 new_state_list[2 * i] = new_data
    #                 new_state_list[2 * i + 1] = new_up
    #                 new_state = tuple(new_state_list)
    #                 print "adding edge from {} to {}".format(state, new_state)
    #                 G.add_edge(state, new_state)

    #     assert num_enabled > 0
    #     if num_enabled == 1:
    #         legitimate_states.append(state)

    # output = open('dijkstra.html', 'w')
    # A = nx.to_agraph(G)
    # for node in G.nodes():
    #     if node in legitimate_states:
    #         A.get_node(node).attr['color'] = 'red'
    # A.layout(prog='dot')
    # A.draw(output, format='svg')
    # output.close()
    pass


def parse_state(state):
    state_line = p.Group(p.Suppress(p.Word(p.alphas + p.nums) + p.Literal('.')) +
                         p.Word(p.alphas + p.nums) +
                         p.Suppress(p.Literal(":")) +
                         p.Word(p.alphas))
    states = p.OneOrMore(state_line)
    the_states = states.parseString(state).asList()
    interesting_states = [(a_state[0], to_bool(a_state[1])) for a_state in the_states
                          if a_state[0].startswith("Up") or a_state[0].startswith("Data")]
    return dict(interesting_states)

def guard_inputs(process_id, number_of_processes):
    if process_id == 0:
        return ['Data0', 'Data1', 'Up1']
    elif process_id == number_of_processes - 1:
        return ['Data{}'.format(number_of_processes - 2), 'Data{}'.format(number_of_processes - 1)]
    else:
        return (['Data{}'.format(i) for i in [process_id - 1, process_id, process_id + 1]] +
                ['Up{}'.format(i) for i in [process_id - 1, process_id, process_id + 1]])
