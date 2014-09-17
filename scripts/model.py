import itertools

import automaton
import util
import z3p


class Model(object):
    def __init__(self, automata, name='model'):
        self.name = name
        self.automata = automata
        self.input_channel_to_automata = {}
        self.output_channel_to_automata = {}
        self.automata_edges = {}
        self.writer = {}
        self.readers = {}
        self.channel_output_transitions = {}
        self.channel_input_transitions = {}
        self.internal_transitions = []
        self.channels = []
        self.liveness_monitors = [a for a in automata if a.has_accept_location()]
        self.safety_monitors = [a for a in automata if a.has_error_location()]

        for a in automata:
            for source, target, data in a.edges(data=True):
                transition = source, target, data
                channel = data['channel']
                if data['kind'] == 'output':
                    if channel not in self.channel_output_transitions:
                        self.channel_output_transitions[channel] = [transition]
                    else:
                        self.channel_output_transitions[channel].append(transition)
                elif data['kind'] == 'input':
                    if a not in self.channel_input_transitions:
                        self.channel_input_transitions[a] = {channel: [transition]}
                    else:
                        if channel not in self.channel_input_transitions[a]:
                            self.channel_input_transitions[a][channel] = [transition]
                        else:
                            self.channel_input_transitions[a][channel].append(transition)
                else:
                    self.internal_transitions.append(transition)
            for c in a.input_channels:
                if c not in self.readers:
                    self.readers[c] = [a]
                else:
                    self.readers[c].append(a)
            for c in a.output_channels:
                self.channels.append(c)
                self.writer[c] = a
            self.automata_edges[a] = a.edges(data=True)

    def variables(self):
        return list(self.variables_iter())

    def variables_iter(self):
        for a in self.automata:
            for v in a.variables:
                yield (v, a.variable_ranges.get(v), a.initials.get(v), a)

    def transitions(self):
        retval = []
        for c in self.channels:
            output_transitions = self.channel_output_transitions[c]
            input_transitions = [self.channel_input_transitions[a][c] for a in self.channel_input_transitions if c in self.channel_input_transitions[a]]
            transitions = [output_transitions] + input_transitions
            for combo in itertools.product(*transitions):
                retval.append(combo)
        retval.extend(self.internal_transitions)
        return retval

    def transitions_by_edge(self, edge):
        retval = []
        for c in self.channels:
            output_transitions = self.channel_output_transitions[c]
            input_transitions = [self.channel_input_transitions[a][c] for a in self.channel_input_transitions if c in self.channel_input_transitions[a]]
            transitions = [output_transitions] + input_transitions
            for combo in itertools.product(*transitions):
                if edge in combo:
                    retval.append(combo)
        return retval

    def transitions_from_state(self, state1, state2, verbose=0):
        possible_transitions = dict([(a, []) for a in self.automata])
        possible_channels = dict([(a, set([])) for a in self.automata])
        possible_internal_transitions = dict([(a, []) for a in self.automata])
        transition_channels = {}
        for a in self.automata:
            if state1[a] == state2[a]:
                possible_transitions[a].append(None)
            for t in a.edges(data=True):
                if a.can_transition(state1[a], state2[a], t):
                    possible_transitions[a].append(t)
                    channel = t[2]['channel']
                    if channel is None:
                        possible_internal_transitions[a].append(t)
                    else:
                        possible_channels[a].add(channel)
        for a in self.automata:
            for c in list(possible_channels[a]):
                assert c in self.readers, "{} not in {}".format(c, self.readers)
                assert c in self.writer, "{} not in {}".format(c, self.writer)
                automata = self.readers[c] + [self.writer[c]]
                if (any(c not in possible_channels[ap]
                        for ap in automata) or
                        any(None not in possible_transitions[ap]
                            for ap in self.automata if ap not in automata)):
                    possible_channels[a].remove(c)
        if any(len(possible_channels[a]) > 0 for a in self.automata):
            c = next(list(possible_channels[a])[0] for a in self.automata
                     if len(possible_channels[a]) > 0)
            if verbose > 0:
                print "Checking channel {}".format(c)
            for t in possible_transitions[self.writer[c]]:
                writer_transition = t
                if verbose > 0:
                    print "Checking writer transition {}".format(t)
                if t is not None and t[2]['channel'] == c:
                    channel_expressions = t[2]['channel_expression']
                    channel_expression_values = [util.evaluate_expression(channel_expression, {}, state1[self.writer[c]], tables=self.writer[c].tables) for channel_expression in channel_expressions]
                    if verbose > 0:
                        print 'Writer tables are {}'.format(self.writer[c].tables)
                        print 'Channel expressions are {}'.format(channel_expressions)
                        print 'Channel expression values are {}'.format(channel_expression_values)
                    reader_transitions = [next(t for t in possible_transitions[a]
                                               if t is not None and t[2]['channel'] == c)
                                          for a in self.readers[c]]
                    writer_matches_readers = True
                    for rt in reader_transitions:
                        if verbose > 0:
                            print "Checking against reader transition {}".format(rt)
                        ra = rt[2]['automaton']
                        updates = rt[2]['update']
                        for update in updates:
                            if c in z3p.variables_in_expression(update[1]):
                                if isinstance(c, z3p.ArrayRef):
                                    selects = util.selects_in_expression(update[1])
                                    new_reader_concrete_memory = dict(state1[ra])
                                    for select in selects:
                                        index = select.arg(1)
                                        new_reader_concrete_memory[c[index]] = channel_expression_values[index.as_long()]
                                    if state2[ra][update[0]] != util.evaluate_expression(update[1], {}, new_reader_concrete_memory):
                                        writer_matches_readers = False
                                else:
                                    new_reader_concrete_memory = dict(state1[ra])
                                    new_reader_concrete_memory[c] = channel_expression_values[0]
                                    s = z3p.Solver()
                                    s.add(z3p.Neq(state2[ra][update[0]], util.evaluate_expression(update[1], {}, new_reader_concrete_memory)))
                                    if s.check() != z3p.unsat:
                                        if verbose:
                                            print "update does not match {}, {}".format(state2[ra][update[0]], util.evaluate_expression(update[1], {}, new_reader_concrete_memory))
                                        writer_matches_readers = False
                    if writer_matches_readers:
                        return [writer_transition] + reader_transitions
        else:
            for a in self.automata:
                other_automata = [ap for ap in self.automata if a != ap]
                if (len(possible_internal_transitions[a]) > 0 and
                        all(None in possible_transitions[ap]
                            for ap in other_automata)):
                    return [possible_internal_transitions[a]]

    def candidate_transitions(self, locations, verbose=0):
        """locations is a map from automaton to location
        """
        # Find candidate output transitions
        output_transitions = {}
        for a, edges in self.automata_edges.items():
            output_edges = [e for e in edges
                            if e[2]['kind'] == 'output' and e[0] == locations[a]]
            output_transitions[a] = output_edges
        # for each possible output transition check that every automaton
        # that can read from the message is at a location where it can do that
        candidates = []
        for automaton, automaton_output_transitions in output_transitions.items():
            for output_edge in automaton_output_transitions:
                if verbose > 0:
                    print "checking output edge {}".format(output_edge)
                channel = output_edge[2]['channel']
                reading_automata = self.readers[channel]
                if all(any(e[0] == locations[a]
                           for e in a.channel_edges[channel])
                       for a in reading_automata):
                    reading_edges = [[(a, e) for e in a.channel_edges[channel]
                                      if e[0] == locations[a]]
                                     for a in reading_automata]
                    for reading_edge_combination in itertools.product(*reading_edges):
                        candidate = [(automaton, output_edge)] + list(reading_edge_combination)
                        candidates.append(candidate)
        for a in self.automata:
            for t in a.edges(data=True):
                if t[2]['kind'] == 'internal' and t[0] == locations[a]:
                    candidates.append([(a, t)])
        return candidates

    def condition_to_resolve_deadlock(self, locations, memory, verbose=1):
        """Locations is a map from automaton to current location
        Memory can contain symbolic values.
        """
        if verbose > 0:
            print "Deadlock locations {}".format(locations)
        candidates = self.candidate_transitions(locations)
        if verbose > 0:
            print "Candidates are {}".format(candidates)
        if len(candidates) == 0:
            return z3p.BoolVal(False)
        else:
            return z3p.Or([z3p.And([a.transition_condition(edge[2]['name'], memory[a])
                                    for a, edge in candidate])
                           for candidate in candidates])

    def __getitem__(self, index):
        if isinstance(index, str):
            automaton_name = index
            return next(automaton for automaton in self.automata
                        if automaton.name == automaton_name)
        elif isinstance(index, automaton.SymbolicAutomaton):
            automaton_name = index.name
            return next(automaton for automaton in self.automata
                        if automaton.name == automaton_name)
        else:
            message = "Indexing of models by {} not implemented".format(type(index))
            raise NotImplementedError(message)
