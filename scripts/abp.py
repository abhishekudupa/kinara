import automaton
import z3p
import util

min_value_message = 0
max_value_message = 5

def sender_client():
    transitions = []
    send = z3p.Int('send')
    for i in range(min_value_message, max_value_message + 1):
        transitions.append(('t{}'.format(i), 'initial', send, [z3p.IntVal(i)], z3p.BoolVal(True), [], 'initial'))
    return automaton.SymbolicAutomaton('sender_client',
                                       ['initial'],
                                       'initial',
                                       transitions,
                                       output_channels=[send],
                                       output_channel_ranges={send: (min_value_message, max_value_message)})


def sender():
    transitions = []
    input_message = z3p.Int('sender_input_message')
    sender_tag = z3p.Int('sender_tag')
    ack_tag = z3p.Int('ack_tag')
    forward_input_channel = z3p.Array('forward_input_channel', z3p.IntSort(), z3p.IntSort())
    backward_output_channel = z3p.Int('backward_output_channel')
    send = z3p.Int('send')
    timeout = util.new_variable('timeout', 'unit')
    transitions.append(('t0', 'initial', send, z3p.BoolVal(True), [(input_message, send)], 'q0'))
    transitions.append(('t1', 'q0', forward_input_channel, [sender_tag, input_message], z3p.BoolVal(True), [], 'q1'))
    transitions.append(('t2', 'q1', timeout, z3p.BoolVal(True), [], 'q0'))
    transitions.append(('t3', 'q1', backward_output_channel, z3p.BoolVal(True), [(ack_tag, backward_output_channel)], 'q2'))
    transitions.append(('t4', 'q2', z3p.Eq(ack_tag, sender_tag), [(sender_tag, z3p.If(z3p.Eq(sender_tag, 0), z3p.IntVal(1), z3p.IntVal(0)))], 'initial'))
    transitions.append(('t5', 'q2', z3p.Neq(ack_tag, sender_tag), [], 'q0'))
    return automaton.SymbolicAutomaton('sender',
                                       ['initial', 'q0', 'q1', 'q2'],
                                       'initial',
                                       transitions,
                                       variables=[input_message, sender_tag, ack_tag],
                                       initial_values=[z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0)],
                                       input_channels=[send, timeout, backward_output_channel],
                                       output_channels=[forward_input_channel],
                                       variable_ranges={input_message: (min_value_message, max_value_message), sender_tag: (0, 1), ack_tag: (0, 1)},
                                       output_channel_ranges={forward_input_channel[1]: (min_value_message, max_value_message), forward_input_channel[0]: (0, 1)})


def timer():
    timeout = util.new_variable('timeout', 'unit')
    transitions = [('t0', 'initial', timeout, [z3p.IntVal(0)], z3p.BoolVal(True), [], 'initial')]
    return automaton.SymbolicAutomaton('timer',
                                       ['initial'],
                                       'initial',
                                       transitions,
                                       output_channels=[timeout])


def receiver():
    forward_output_channel = z3p.Array('forward_output_channel', z3p.IntSort(), z3p.IntSort())
    backward_input_channel = z3p.Int('backward_input_channel')
    receive = z3p.Int('receive')
    input_tag = z3p.Int('receiver_tag')
    input_data = z3p.Int('receiver_data')
    expected_tag = z3p.Int('expected_tag')
    transitions = []
    transitions.append(('t0', 'initial', forward_output_channel, z3p.BoolVal(True), [(input_tag, forward_output_channel[0]), (input_data, forward_output_channel[1])], 'q0'))
    transitions.append(('t1', 'q0', receive, [input_data], z3p.Eq(input_tag, expected_tag), [], 'q1'))
    transitions.append(('t2', 'q1', backward_input_channel, [expected_tag], z3p.BoolVal(True), [(expected_tag, z3p.If(z3p.Eq(expected_tag, 0), z3p.IntVal(1), z3p.IntVal(0)))], 'initial'))
    transitions.append(('t3', 'q0', z3p.Neq(input_tag, expected_tag), [], 'q2'))
    transitions.append(('t4', 'q2', backward_input_channel, [z3p.If(z3p.Eq(expected_tag, 0), z3p.IntVal(1), z3p.IntVal(0))], z3p.BoolVal(True), [], 'initial'))
    return automaton.SymbolicAutomaton('receiver',
                                       ['initial', 'q0', 'q1', 'q2'],
                                       'initial',
                                       transitions,
                                       input_channels=[forward_output_channel],
                                       output_channels=[receive, backward_input_channel],
                                       variables=[input_tag, input_data, expected_tag],
                                       initial_values=[z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0)],
                                       variable_ranges={input_tag: (0, 1), input_data: (min_value_message, max_value_message), expected_tag: (0, 1)},
                                       output_channel_ranges={receive: (min_value_message, max_value_message), backward_input_channel: (0, 1)},
                                       justice=['t3', 't1'])


def receiver_client():
    transitions = []
    receive = z3p.Int('receive')
    transitions.append(('t0', 'initial', receive, z3p.BoolVal(True), [], 'initial'))
    return automaton.SymbolicAutomaton('receiver_client',
                                       ['initial'],
                                       'initial',
                                       transitions,
                                       input_channels=[receive])


def safety_monitor():
    safety_data_sent = z3p.Int('safety_data_sent')
    safety_data_received = z3p.Int('safety_data_received')
    variable_ranges = {safety_data_sent: (min_value_message, max_value_message),
                       safety_data_received: (min_value_message, max_value_message)}
    send = z3p.Int('send')
    receive = z3p.Int('receive')
    transitions = []
    transitions.append(('t0', 'waiting_for_send', send, z3p.BoolVal(True), [(safety_data_sent, send)], 'waiting_for_receive'))
    transitions.append(('t1', 'waiting_for_receive', receive, z3p.BoolVal(True), [(safety_data_received, receive)], 'check_received_data'))
    transitions.append(('t2', 'check_received_data', z3p.Neq(safety_data_sent, safety_data_received), [], 'error'))
    transitions.append(('t3', 'check_received_data', z3p.Eq(safety_data_sent, safety_data_received), [], 'waiting_for_send'))
    transitions.append(('t4', 'waiting_for_send', receive, z3p.BoolVal(True), [], 'error'))
    transitions.append(('t5', 'waiting_for_receive', send, z3p.BoolVal(True), [], 'error'))
    transitions.append(('t6', 'error', send, z3p.BoolVal(True), [], 'error'))
    transitions.append(('t6', 'error', receive, z3p.BoolVal(True), [], 'error'))
    return automaton.SymbolicAutomaton('safety_monitor',
                                       ['waiting_for_send', 'waiting_for_receive', 'error', 'check_received_data'],
                                       'waiting_for_send',
                                       transitions,
                                       [safety_data_sent, safety_data_received],
                                       [z3p.IntVal(0), z3p.IntVal(0)],
                                       input_channels=[send, receive],
                                       variable_ranges=variable_ranges)


def lossless_tag_data_channel(name):
    input_channel = z3p.Array('{}_input_channel'.format(name), z3p.IntSort(), z3p.IntSort())
    output_channel = z3p.Array('{}_output_channel'.format(name), z3p.IntSort(), z3p.IntSort())
    tag = z3p.Int('{}_channel_tag'.format(name))
    data = z3p.Int('{}_channel_data'.format(name))
    transitions = []
    transitions.append(('t0', 'empty', input_channel, z3p.BoolVal(True), [(tag, input_channel[0]), (data, input_channel[1])], 'full'))
    transitions.append(('t1', 'full', output_channel, [tag, data], z3p.BoolVal(True), [], 'empty'))
    transitions.append(('t2', 'full', input_channel, z3p.BoolVal(True), [], 'full'))
    return automaton.SymbolicAutomaton('{}_tag_data_channel'.format(name),
                                       ['empty', 'full'],
                                       'empty',
                                       transitions,
                                       input_channels=[input_channel],
                                       output_channels=[output_channel],
                                       variables=[tag, data],
                                       initial_values=[z3p.IntVal(0), z3p.IntVal(0)],
                                       variable_ranges={tag: (0, 1), data: (min_value_message, max_value_message)})


def lossy_duplicating_tag_data_channel(name):
    input_channel = z3p.Array('{}_input_channel'.format(name), z3p.IntSort(), z3p.IntSort())
    output_channel = z3p.Array('{}_output_channel'.format(name), z3p.IntSort(), z3p.IntSort())
    tag = z3p.Int('{}_channel_tag'.format(name))
    data = z3p.Int('{}_channel_data'.format(name))
    transitions = []
    transitions.append(('t0', 'empty', input_channel, z3p.BoolVal(True), [(tag, input_channel[0]), (data, input_channel[1])], 'full'))
    transitions.append(('t1', 'full', output_channel, [tag, data], z3p.BoolVal(True), [], 'empty'))
    transitions.append(('t2', 'full', input_channel, z3p.BoolVal(True), [], 'full'))
    transitions.append(('t_duplication', 'full', output_channel, [tag, data], z3p.BoolVal(True), [], 'full'))
    transitions.append(('t_loss', 'empty', input_channel, z3p.BoolVal(True), [], 'empty'))
    return automaton.SymbolicAutomaton('{}_tag_data_channel'.format(name),
                                       ['empty', 'full'],
                                       'empty',
                                       transitions,
                                       input_channels=[input_channel],
                                       output_channels=[output_channel],
                                       variables=[tag, data],
                                       initial_values=[z3p.IntVal(0), z3p.IntVal(0)],
                                       variable_ranges={tag: (0, 1), data: (min_value_message, max_value_message)},
                                       compassion=['t1', 't0'])



def forward_channel():
    return lossy_duplicating_tag_data_channel('forward')


def lossless_tag_channel(name):
    input_channel = z3p.Int('{}_input_channel'.format(name))
    output_channel = z3p.Int('{}_output_channel'.format(name))
    tag = z3p.Int('{}_channel_tag'.format(name))
    transitions = []
    transitions.append(('t0', 'empty', input_channel, z3p.BoolVal(True), [(tag, input_channel)], 'full'))
    transitions.append(('t1', 'full', output_channel, [tag], z3p.BoolVal(True), [], 'empty'))
    transitions.append(('t2', 'full', input_channel, z3p.BoolVal(True), [], 'full'))
    return automaton.SymbolicAutomaton('{}_tag_channel'.format(name),
                                       ['empty', 'full'],
                                       'empty',
                                       transitions,
                                       input_channels=[input_channel],
                                       output_channels=[output_channel],
                                       variables=[tag],
                                       initial_values=[z3p.IntVal(0)],
                                       variable_ranges={tag: (0, 1)},
                                       output_channel_ranges={output_channel: (0, 1)})


def lossy_duplicating_tag_channel(name):
    input_channel = z3p.Int('{}_input_channel'.format(name))
    output_channel = z3p.Int('{}_output_channel'.format(name))
    tag = z3p.Int('{}_channel_tag'.format(name))
    transitions = []
    transitions.append(('t0', 'empty', input_channel, z3p.BoolVal(True), [(tag, input_channel)], 'full'))
    transitions.append(('t_loss', 'empty', input_channel, z3p.BoolVal(True), [], 'empty'))
    transitions.append(('t1', 'full', output_channel, [tag], z3p.BoolVal(True), [], 'empty'))
    transitions.append(('t_duplicating', 'full', output_channel, [tag], z3p.BoolVal(True), [], 'full'))
    transitions.append(('t2', 'full', input_channel, z3p.BoolVal(True), [], 'full'))
    return automaton.SymbolicAutomaton('{}_tag_channel'.format(name),
                                       ['empty', 'full'],
                                       'empty',
                                       transitions,
                                       input_channels=[input_channel],
                                       output_channels=[output_channel],
                                       variables=[tag],
                                       initial_values=[z3p.IntVal(0)],
                                       variable_ranges={tag: (0, 1)},
                                       output_channel_ranges={output_channel: (0, 1)},
                                       compassion=['t0', 't1'])


def backward_channel():
    return lossy_duplicating_tag_channel('backward')


def liveness_monitor1():
    send = z3p.Int('send')
    receive = z3p.Int('receive')
    transitions = []
    transitions.append(('t1', 'initial', send, z3p.BoolVal(True), [], 'accept'))
    transitions.append(('t2', 'initial', send, z3p.BoolVal(True), [], 'initial'))
    transitions.append(('t3', 'initial', receive, z3p.BoolVal(True), [], 'initial'))
    transitions.append(('t4', 'accept', send, z3p.BoolVal(True), [], 'accept'))
    transitions.append(('t5', 'accept', receive, z3p.BoolVal(True), [], 'final'))
    transitions.append(('t6', 'final', send, z3p.BoolVal(True), [], 'final'))
    transitions.append(('t7', 'final', receive, z3p.BoolVal(True), [], 'final'))
    return automaton.SymbolicAutomaton('liveness_monitor1',
                                       ['initial', 'accept', 'final'],
                                       'initial',
                                       transitions,
                                       input_channels=[send, receive])


def liveness_monitor2():
    send = z3p.Int('send')
    receive = z3p.Int('receive')
    transitions = []
    transitions.append(('t1', 'initial', receive, z3p.BoolVal(True), [], 'accept'))
    transitions.append(('t2', 'initial', receive, z3p.BoolVal(True), [], 'initial'))
    transitions.append(('t3', 'initial', send, z3p.BoolVal(True), [], 'initial'))
    transitions.append(('t4', 'accept', receive, z3p.BoolVal(True), [], 'accept'))
    transitions.append(('t5', 'accept', send, z3p.BoolVal(True), [], 'final'))
    transitions.append(('t6', 'final', receive, z3p.BoolVal(True), [], 'final'))
    transitions.append(('t7', 'final', send, z3p.BoolVal(True), [], 'final'))
    return automaton.SymbolicAutomaton('liveness_monitor2',
                                       ['initial', 'accept', 'final'],
                                       'initial',
                                       transitions,
                                       input_channels=[receive, send])


automata = [sender_client(), sender(), forward_channel(), timer(), receiver(), receiver_client(), safety_monitor(), backward_channel(), liveness_monitor1(), liveness_monitor2()]
