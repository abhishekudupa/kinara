import automaton
import util
import z3p

from msi_channels import *

def cache(c, num_caches=1, num_values=1, num_addresses=1, num_directories=1):
    transitions = []
    variables = []
    variable_ranges = {}
    DataBlk = z3p.Int('DataBlk_{}'.format(c))
    AckCounter = z3p.Int('AckCounter_{}'.format(c))
    PendingWrite = z3p.Int('PendingWrite_{}'.format(c))
    FwdToCache = z3p.Int('FwdToCache_{}'.format(c))
    NumAcksTemp = z3p.Int('NumAcksTemp_{}'.format(c))
    variable_ranges = {DataBlk: (0, num_values - 1),
                       AckCounter: (-num_caches, num_caches),
                       PendingWrite: (0, num_values - 1),
                       FwdToCache: (0, num_caches - 1),
                       NumAcksTemp: (0, num_caches)}
    variables = [DataBlk, AckCounter, PendingWrite, FwdToCache, NumAcksTemp]
    initial_values = [ZERO, ZERO, ZERO, ZERO, ZERO]
    # ??? initial_values

    # input messages
    input_channels = []
    for d in range(num_directories):
        for a in range(num_addresses):
            input_channels.extend([LDMsg[c][d][a], STMsg[c][d][a], EVMsg[c][d][a], FwdGetXMsgP[c][d][a], FwdGetSMsgP[c][d][a], DataMsgD2CP[c][d][a], WBAckMsgP[c][d][a]])
    for c2 in range(num_caches):
        if c2 != c:
            for d in range(num_directories):
                for a in range(num_addresses):
                    input_channels.extend([DataMsgC2CP[c2][c][d][a], InvAckMsgP[c2][c][d][a]])

    # output messages
    output_channels = []
    for d in range(num_directories):
        for a in range(num_addresses):
            output_channels.extend([LDAckMsg[c][d][a], STAckMsg[c][d][a], EVAckMsg[c][d][a], UnblockSMsg[c][d][a], UnblockEMsg[c][d][a], GetXMsg[c][d][a], GetSMsg[c][d][a], WBMsg[c][d][a]])
    for c2 in range(num_caches):
        if c2 != c:
            output_channels.extend([DataMsgC2C[c][c2][d][a], InvAckMsg[c][c2][d][a]])

    for d in range(num_directories):
        for a in range(num_addresses):
            # // Transitions from I for Ext events
            # C_I on LDMsg[c][d][a] {} -> send GetSMsg[c][d][a] {} -> C_IS;
            # C_I on STMsg[c][d][a] (v) { PendingWrite := v; } ->
            # send GetXMsg[c][d][a] {} -> C_IM;
            # C_I on EVMsg[c][d][a] {} -> send EVAckMsg[c][d][a] -> C_I;
            transitions.append((None, 'C_I', LDMsg[c][d][a], TRUE, [], 'C_I2'))
            transitions.append((None, 'C_I2', GetSMsg[c][d][a], [], TRUE, [], 'C_IS'))
            transitions.append((None, 'C_I', STMsg[c][d][a], TRUE, [(PendingWrite, STMsg[c][d][a])], 'C_I3'))
            transitions.append((None, 'C_I3', GetXMsg[c][d][a], [], TRUE, [], 'C_IM'))
            transitions.append((None, 'C_I', EVMsg[c][d][a], TRUE, [], 'C_I4'))
            transitions.append((None, 'C_I4', EVAckMsg[c][d][a], [], TRUE, [], 'C_I'))

            # // Transitions for I on Fwd events
            # C_I on FwdGetSMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] { FwdToCache := undef; } -> C_I;
            transitions.append((None, 'C_I', FwdGetSMsgP[c][d][a], TRUE, [(FwdToCache, FwdGetSMsgP[c][d][a])], 'C_I5'))
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_I5', InvAckMsg[c][other_c][d][a], [], z3p.Eq(INT(other_c), FwdToCache), [(FwdToCache, ZERO)], 'C_I'))


            # // Transitions from S on Ext Events
            # C_S on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) -> C_S;
            transitions.append((None, 'C_S', LDMsg[c][d][a], TRUE, [], 'C_S2'))
            transitions.append((None, 'C_S2', LDAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_S'))

            # C_S on STMsg[c][d][a] (v) { PendingWrite := v; } ->
            # send GetXMsg[c][d][a] {} -> C_SM;
            transitions.append((None, 'C_S', STMsg[c][d][a], TRUE, [(PendingWrite, STMsg[c][d][a])], 'C_S3'))
            transitions.append((None, 'C_S3', GetXMsg[c][d][a], [], TRUE, [], 'C_SM'))

            # C_S on EVMsg[c][d][a] {} ->
            # send EVAckMsg[c][d][a] { DataBlk := undef; } -> C_I;
            transitions.append((None, 'C_S', EVMsg[c][d][a], TRUE, [], 'C_S4'))
            transitions.append((None, 'C_S4', EVAckMsg[c][d][a], [], TRUE, [(DataBlk, ZERO)], 'C_I'))

            # // Transitions from S on Fwd events
            # C_S on FwdGetXMsg[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] { DataBlk := undef } -> C_I;
            transitions.append((None, 'C_S', FwdGetXMsgP[c][d][a], TRUE, [(FwdToCache, FwdGetXMsgP[c][d][a]), (DataBlk, ZERO)], 'C_S5'))
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_S5', InvAckMsg[c][other_c][d][a], [], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, ZERO)], 'C_I'))

            # // Transitions from M on Ext events
            # C_M on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_M;
            transitions.append((None, 'C_M', LDMsg[c][d][a], TRUE, [], 'C_M2'))
            transitions.append((None, 'C_M2', LDAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_M'))
            # C_M on STMsg[c][d][a] (v) { DataBlk := v; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            transitions.append((None, 'C_M', STMsg[c][d][a], TRUE, [(DataBlk, STMsg[c][d][a])], 'C_M3'))
            transitions.append((None, 'C_M3', STAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_M'))

            # C_M on EVMsg[c][d][a] {} -> send WBMsg[c][d][a] (DataBlk) {} -> C_II;
            transitions.append((None, 'C_M', EVMsg[c][d][a], TRUE, [], 'C_M4'))
            transitions.append((None, 'C_M4', WBMsg[c][d][a], [DataBlk], TRUE, [], 'C_II'))

            # // Transitions from M on Fwd events
            # C_M on FwdGetSMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send DataMsgC2C[c][FwdToCache][d][a] (DataBlk) { FwdToCache := undef; } -> C_S;
            transitions.append((None, 'C_M', FwdGetSMsgP[c][d][a], TRUE, [(FwdToCache, FwdGetSMsgP[c][d][a])], 'C_M5'))
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_M5', DataMsgC2C[c][other_c][d][a], [DataBlk], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, ZERO)], 'C_S'))

            # C_M on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send DataMsgC2C[c][FwdToCache][d][a] (DataBlk)
            # { FwdToCache := undef; DataBlk := undef; } -> C_I;
            transitions.append((None, 'C_M', FwdGetXMsgP[c][d][a], TRUE, [(FwdToCache, FwdGetXMsgP[c][d][a])], 'C_M6'))
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_M6', DataMsgC2C[c][other_c][d][a], [DataBlk], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, ZERO), (DataBlk, ZERO)], 'C_I'))

            # // Transitions from C_IM on Rsp Events
            # C_IM on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {FwdToCache := undef; } -> C_IM;
            transitions.append((None, 'C_IM', FwdGetXMsgP[c][d][a], TRUE, [(FwdToCache, FwdGetXMsgP[c][d][a])], 'C_IM2'))
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_IM2', InvAckMsg[c][other_c][d][a], [DataBlk], z3p.Eq(INT(other_c), FwdToCache), [(FwdToCache, ZERO)], 'C_IM'))
            # // Case when another cache gives me data
            # foreach c2 in CacheIDType (!= c2 c) {
            # C_IM on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> C_M;
            # }
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_IM', DataMsgC2CP[other_c][c][d][a], TRUE, [(DataBlk, DataMsgC2CP[other_c][c][d][a])], 'C_M'))

            # // Case when data comes from dir ==> Must wait for acks
            # C_IM on DataMsgD2C'[c][d][a] (v, NumAcks)
            # if (!= AckCounter NumAcks)
            # { AckCounter := (- AckCounter NumAcks); DataBlk := v; } -> C_SM;
            # if (= AckCounter NumAcks)
            # { AckCounter := 0; DataBlk := v; } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } -> C_M;
            transitions.append((None, 'C_IM', DataMsgD2CP[c][d][a], TRUE, [(DataBlk, DataMsgD2CP[c][d][a][0]), (NumAcksTemp, DataMsgD2CP[c][d][a][1])], 'C_IM3'))
            transitions.append((None, 'C_IM3', z3p.Neq(AckCounter, NumAcksTemp), [(AckCounter, AckCounter - NumAcksTemp), (NumAcksTemp, ZERO)], 'C_SM'))
            transitions.append((None, 'C_IM3', z3p.Eq(AckCounter, NumAcksTemp), [(AckCounter, ZERO), (NumAcksTemp, ZERO)], 'C_IM4'))
            transitions.append((None, 'C_IM4', STAckMsg[c][d][a], [PendingWrite], TRUE, [(DataBlk, PendingWrite), (PendingWrite, ZERO)], 'C_IM5'))
            transitions.append((None, 'C_IM5', UnblockEMsg[c][d][a], [ZERO], TRUE, [], 'C_M'))

            # foreach c2 in CacheIDType (!= c2 c) {
            #     C_IM on InvAckMsg[c2][c][d][a] { AckCounter := (+ AckCounter 1); } -> C_IM;
            # }
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_IM', InvAckMsgP[other_c][c][d][a], TRUE, [(AckCounter, AckCounter + 1)], 'C_IM'))

            # // Transitions from C_SM on Ext Events
            # C_SM on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_SM;
            transitions.append((None, 'C_SM', LDMsg[c][d][a], TRUE, [], 'C_SM2'))
            transitions.append((None, 'C_SM2', LDAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_SM'))

            # // Transitions from C_SM on Rsp Events
            # C_SM on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {} -> C_SM;
            transitions.append((None, 'C_SM', FwdGetXMsgP[c][d][a], TRUE, [(FwdToCache, FwdGetXMsgP[c][d][a])], 'C_SM3'))
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_SM3', InvAckMsg[c][other_c][d][a], [], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [], 'C_IM'))


            # // Case where data comes from dir ==> Must wait for acks
            # C_SM on DataMsgD2C'[c][d][a] (v, NumAcks)

            transitions.append((None, 'C_SM', DataMsgD2CP[c][d][a], TRUE, [(NumAcksTemp, DataMsgD2CP[c][d][a][1])], 'C_SM4'))

            # if (= NumAcks AckCounter) { AckCounter := 0; } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            # if (!= NumAcks AckCounter) { AckCounter := (- AckCounter NumAcks) } -> C_SM;
            transitions.append((None, 'C_SM4', UnblockEMsg[c][d][a], [ZERO], z3p.Eq(NumAcksTemp, AckCounter), [(DataBlk, PendingWrite), (PendingWrite, ZERO), (NumAcksTemp, ZERO), (AckCounter, ZERO)], 'C_SM5'))
            transitions.append((None, 'C_SM5', STAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_M'))
            transitions.append((None, 'C_SM4', z3p.Neq(NumAcksTemp, AckCounter), [(AckCounter, AckCounter - NumAcksTemp), (NumAcksTemp, ZERO)], 'C_SM'))

            # foreach c2 in CacheIDType (!= c2 c) {
            #     C_SM on InvAckMsg'[c2][c][d][a]
            # if (= AckCounter (- 1)) { AckCounter := 0; } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            # if (!= AckCounter (- 1)) { AckCounter := (+ AckCounter 1); } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            # }
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_SM', InvAckMsgP[other_c][c][d][a], z3p.Eq(AckCounter, -ONE), [(AckCounter, ZERO), (DataBlk, PendingWrite), (PendingWrite, ZERO)], 'C_SM6'))
                    transitions.append((None, 'C_SM6', UnblockEMsg[c][d][a], [DataBlk], TRUE, [], 'C_SM7'))
                    transitions.append((None, 'C_SM7', STAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_M'))

                    transitions.append((None, 'C_SM', InvAckMsgP[other_c][c][d][a], z3p.Neq(AckCounter, -ONE), [(AckCounter, AckCounter + 1)], 'C_SM'))
                    # transitions.append((None, 'C_SM6', UnblockEMsg[c][d][a], [DataBlk], TRUE, [], 'C_SM7'))
                    # transitions.append((None, 'C_SM7', STAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_M'))

                    # transitions.append((None, 'C_SM', InvAckMsgP[other_c][c][d][a], TRUE, [], 'C_SM6'))
                    # transitions.append((None, 'C_SM6', UnblockEMsg[c][d][a], [ZERO], z3p.Eq(AckCounter, - ONE), [(AckCounter, ZERO), (DataBlk, PendingWrite), (PendingWrite, ZERO)], 'C_SM6'))
                    # transitions.append((None, 'C_SM7', STAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_M'))

                    # transitions.append((None, 'C_SM6', UnblockEMsg[c][d][a], [], z3p.Neq(AckCounter, ZERO - ONE), [(AckCounter, AckCounter + 1), (DataBlk, PendingWrite), (PendingWrite, ZERO)], 'C_SM7'))
                    # transitions.append((None, 'C_SM8', STAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_M'))

            # // Transitions from IS on Ext Events
            # C_IS on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {} -> C_IS;
            transitions.append((None, 'C_IS', FwdGetXMsgP[c][d][a], TRUE, [(FwdToCache, FwdGetXMsgP[c][d][a])], 'C_IS2'))
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_IS2', InvAckMsg[c][other_c][d][a], [], z3p.Eq(INT(other_c), FwdToCache), [], 'C_IS'))

            # // Transitions on IS on Rsp Events
            # C_IS on DataMsgD2C'[c][d][a] (v, _) { DataBlk := v; } ->
            # send UnblockSMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_S;
            transitions.append((None, 'C_IS', DataMsgD2CP[c][d][a], TRUE, [(DataBlk, DataMsgD2CP[c][d][a][0])], 'C_IS3'))
            transitions.append((None, 'C_IS3', UnblockSMsg[c][d][a], [ZERO], TRUE, [], 'C_IS4'))
            transitions.append((None, 'C_IS4', LDAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_S'))

            # foreach c2 in CacheIDType (!= c2 c) {
            # C_IS on DataMsgC2C'[c2][c][d][a] (v) -> { DataBlk := v; } ->
            # send UnblockSMsg[c][d][a] -> send LDAckMsg[c][d][a] (DataBlk) -> C_S;
            # }
            for other_c in range(num_caches):
                if other_c != c:
                    transitions.append((None, 'C_IS', DataMsgC2CP[other_c][c][d][a], TRUE, [(DataBlk, DataMsgC2CP[other_c][c][d][a])], 'C_IS3'))
                    # transitions.append((None, 'C_IS4', UnblockSMsg[c][d][a], [ZERO], TRUE, [], 'C_IS5'))
                    # transitions.append((None, 'C_IS5', LDAckMsg[c][d][a], [DataBlk], TRUE, [], 'C_S'))

            # // Transitions on II on Rsp Events
            # C_II on WBAckMsg'[c][d][a] -> C_I;
            # send EVAck and then go to C_I
            transitions.append((None, 'C_II', WBAckMsgP[c][d][a], TRUE, [], 'C_II2'))
            transitions.append((None, 'C_II2', EVAckMsg[c][d][a], [ZERO], TRUE, [], 'C_I'))


            # C_II on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] -> C_I;
            transitions.append((None, 'C_II', FwdGetXMsgP[c][d][a], TRUE, [(FwdToCache, FwdGetXMsgP[c][d][a])], 'C_I5'))
            # for other_c in range(num_caches):
            #     if other_c != c:
            #         transitions.append((None, 'C_II2', InvAckMsg[c][other_c][d][a], [], z3p.Eq(FwdToCache, other_c), [(FwdToCache, ZERO)], 'C_I'))


            # C_II on FwdGetSMsg'[c][d][a] (_) -> I;
            # }
            transitions.append((None, 'C_II', FwdGetSMsgP[c][d][a], TRUE, [], 'C_I'))


    locations = ['C_I', 'C_S', 'C_M', 'C_IM', 'C_SM', 'C_IS', 'C_II']
    locations.extend(['C_I2', 'C_I3', 'C_I4', 'C_I5'])
    locations.extend(['C_S2', 'C_S3', 'C_S4', 'C_S5'])
    locations.extend(['C_M2', 'C_M3', 'C_M4', 'C_M5', 'C_M6'])
    locations.extend(['C_IM2', 'C_IM3', 'C_IM4', 'C_IM5'])
    locations.extend(['C_SM2', 'C_SM3', 'C_SM4', 'C_SM5', 'C_SM6', 'C_SM7', 'C_SM8'])
    locations.extend(['C_IS2', 'C_IS3', 'C_IS4']) # , 'C_IS5'])
    locations.extend(['C_II2'])

    return automaton.SymbolicAutomaton('cache{}'.format(c),
                                       locations,
                                       'C_I',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=input_channels,
                                       output_channels=output_channels,
                                       variable_ranges=variable_ranges)


def directory(d, num_caches=2, num_values=1, num_addresses=1):
    transitions = []
    variables = []
    variable_ranges = {}
    DataBlk = z3p.Int('DataBlk')
    ActiveId = z3p.Int('ActiveId')
    Sharers = {}
    for cache_id in range(num_caches):
        Sharers[cache_id] = z3p.Int('Sharers_{}'.format(cache_id))
    NumSharers = z3p.Int('NumSharers')
    Owner = z3p.Int('Owner')

    variable_ranges = {DataBlk: (0, num_values - 1),
                       ActiveId: (0, num_caches - 1),
                       NumSharers: (0, num_caches),
                       Owner: (0, num_caches - 1)}
    for cache_id in range(num_caches):
        variable_ranges[Sharers[cache_id]] = (0, 1)

    # Do i need numsharers

    variables = [DataBlk, ActiveId, NumSharers, Owner] + [Sharers[cache_id] for cache_id in range(num_caches)]

    initial_values = [ZERO, ZERO, ZERO, ZERO] + num_caches * [ZERO]

    locations = ['D_I', 'D_S', 'D_M', 'D_BUSY', 'D_DATA', 'D_PENDING_UNBLOCK_E', 'D_BUSY_DATA']

    # input messages
    input_channels = []
    for c in range(num_caches):
        for a in range(num_addresses):
            input_channels.extend([GetXMsgP[c][d][a], GetSMsgP[c][d][a], WBMsgP[c][d][a], UnblockSMsgP[c][d][a], UnblockEMsgP[c][d][a]])
            for other_c in range(num_caches):
                if other_c != c:
                    input_channels.append(DataMsgC2CP[c][other_c][d][a])

    output_channels = []
    # output messages
    for c in range(num_caches):
        for a in range(num_addresses):
            output_channels.extend([FwdGetXMsg[c][d][a], FwdGetSMsg[c][d][a], DataMsgD2C[c][d][a], WBAckMsg[c][d][a]])

    for c in range(num_caches):
        for a in range(num_addresses):
            # // Transitions from I
            #  D_I on GetXMsg'[c][d][a] { ActiveId := c; } ->
            #  send DataMsgD2C[c][d][a] (DataBlk, 0) -> D_BUSY;
            target = 'D_I2_{}'.format(c)
            transitions.append((None, 'D_I', GetXMsgP[c][d][a], TRUE, [(ActiveId, INT(c))], target))
            transitions.append((None, target, DataMsgD2C[c][d][a], [DataBlk, ZERO], TRUE, [], 'D_BUSY'))
            locations.append(target)
            #  D_I on GetSMsg'[c][d][a] { ActiveId := c; } ->
            #  send DataMsgD2C[c][d][a] (DataBlk, 0) -> D_BUSY;
            target = 'D_I3_{}'.format(c)
            transitions.append((None, 'D_I', GetSMsgP[c][d][a], TRUE, [(ActiveId, INT(c))], target))
            transitions.append((None, target, DataMsgD2C[c][d][a], [DataBlk, ZERO], TRUE, [], 'D_BUSY'))
            locations.append(target)
            #  // Transitions from S
            #  D_S on GetXMsg'[c][d][a] { ActiveId := c; } ->
            #  send DataMsgD2C[c][d][a] (DataBlk, NumSharers) {} ->
            #  foreach c2 in CacheIDType {
            #      if (Sharers[c2]) send FwdGetXMsg[c2][d][a] (c) {};
            #      if (not Sharers[c2]) pass {};
            #  } -> D_BUSY;
            target = 'D_S2_{}'.format(c)
            target2 = 'D_S3_{}'.format(c)
            transitions.append((None, 'D_S', GetXMsgP[c][d][a], TRUE, [(ActiveId, INT(c))], target))
            transitions.append((None, target, DataMsgD2C[c][d][a], [DataBlk, NumSharers - 1], z3p.Eq(Sharers[c], ONE), [], target2))
            transitions.append((None, target, DataMsgD2C[c][d][a], [DataBlk, NumSharers], z3p.Eq(Sharers[c], ZERO), [], target2))
            locations.extend([target, target2])

            last_state = 'D_S3_{}'.format(c)
            other_caches = [other_c for other_c in range(num_caches) if c != other_c]
            for i, other_c in enumerate(other_caches):
                if other_c != c:
                    if i + 1 == len(other_caches):
                        target = 'D_BUSY'
                    else:
                        target = 'D_S3_{}_{}'.format(c, other_c)
                    source = last_state
                    transitions.append((None, source, FwdGetXMsg[other_c][d][a], [z3p.IntVal(c)], z3p.Eq(Sharers[other_c], ONE), [], target))
                    transitions.append((None, source, z3p.Neq(Sharers[other_c], ONE), [], target))
                    locations.extend([target])
                    last_state = target

            #  D_S on GetSMsg'[c][d][a] { ActiveId := c; } ->
            #  send DataMsgD2C[c][d][a] (DataBlk, 0) {} -> D_BUSY;
            target = 'D_S4_{}'.format(c)
            transitions.append((None, 'D_S', GetSMsgP[c][d][a], TRUE, [(ActiveId, INT(c))], target))
            transitions.append((None, target, DataMsgD2C[c][d][a], [DataBlk, ZERO], TRUE, [], 'D_BUSY'))
            locations.append(target)
            #  // Transitions from M
            #  D_M on GetXMsg'[c][d][a] { ActiveId := c; } ->
            #  send FwdGetXMsg[Owner][d][a] (ActiveId) {} -> D_BUSY;
            target = 'D_M2_{}'.format(c)
            transitions.append((None, 'D_M', GetXMsgP[c][d][a], TRUE, [(ActiveId, INT(c))], target))
            for c2 in range(num_caches):
                transitions.append((None, target, FwdGetXMsg[c2][d][a], [ActiveId], z3p.Eq(INT(c2), Owner), [], 'D_BUSY'))
            locations.append(target)
            #  D_M on GetSMsg'[c][d][a] { ActiveId := c; } ->
            #  send FwdGetSMsg[Owner][d][a] (ActiveId) {} -> D_BUSY_DATA;
            target = 'D_M3_{}'.format(c)
            transitions.append((None, 'D_M', GetSMsgP[c][d][a], TRUE, [(ActiveId, INT(c))], target))
            for c2 in range(num_caches):
                transitions.append((None, target, FwdGetSMsg[c2][d][a], [ActiveId], z3p.Eq(INT(c2), Owner), [], 'D_BUSY_DATA'))
            locations.append(target)
            #  D_M on WBMsg'[c][d][a] (v) { DataBlk := v; Sharers[c] := false; } ->
            #  send WBAckMsg[c][d][a] {} -> D_I;
            target = 'D_M4_{}'.format(c)
            transitions.append((None, 'D_M', WBMsgP[c][d][a], z3p.Eq(Sharers[c], ONE), [(DataBlk, WBMsgP[c][d][a]), (Sharers[c], ZERO), (NumSharers, NumSharers - 1)], target))
            # debug
            transitions.append((None, 'D_M', WBMsgP[c][d][a], z3p.Neq(Sharers[c], ONE), [(DataBlk, WBMsgP[c][d][a])], target))

            transitions.append((None, target, WBAckMsg[c][d][a], [ZERO], TRUE, [(ActiveId, ZERO)], 'D_I'))
            locations.append(target)
            #  // Transitions from BUSY
            #  D_BUSY on WBMsg'[c][d][a] (v)
            #  if (= c ActiveId) { DataBlk := v; } -> D_PENDING_UNBLOCK_E;
            #  if (!= c ActiveId) { Sharers[c] := false; DataBlk := v; } ->
            #  send DataMsgD2C[ActiveId][d][a] (DataBlk, 0) -> D_BUSY;
            transitions.append((None, 'D_BUSY', WBMsgP[c][d][a], z3p.Eq(INT(c), ActiveId), [(DataBlk, WBMsgP[c][d][a])], 'D_PENDING_UNBLOCK_E'))
            target = 'D_BUSY2_{}'.format(c)
            transitions.append((None, 'D_BUSY', WBMsgP[c][d][a], z3p.And(z3p.Neq(INT(c), ActiveId), z3p.Eq(Sharers[c], ONE)), [(Sharers[c], ZERO), (NumSharers, NumSharers - 1), (DataBlk, WBMsgP[c][d][a])], target))
            # debug
            transitions.append((None, 'D_BUSY', WBMsgP[c][d][a], z3p.And(z3p.Neq(INT(c), ActiveId), z3p.Eq(Sharers[c], ZERO)), [(DataBlk, WBMsgP[c][d][a])], target))
            for c2 in range(num_caches):
                transitions.append((None, target, DataMsgD2C[c2][d][a], [DataBlk, ONE], z3p.Eq(INT(c2), ActiveId), [], 'D_BUSY'))
            locations.append(target)
            #  D_BUSY on UnblockEMsg'[c][d][a] { Sharers[c] := true; Owner := c; } -> D_M;
            transitions.append((None, 'D_BUSY', UnblockEMsgP[c][d][a], TRUE, [(Sharers[c], ONE), (NumSharers, ONE), (Owner, INT(c)), (ActiveId, ZERO)] + [(Sharers[other_c], ZERO) for other_c in range(num_caches) if other_c != c], 'D_M'))
            #  D_BUSY on UnblockSMsg'[c][d][a] { Sharers[c] := true; Owner := undef; } -> D_S;
            transitions.append((None, 'D_BUSY', UnblockSMsgP[c][d][a], z3p.Eq(Sharers[c], ZERO), [(Sharers[c], ONE), (NumSharers, NumSharers + 1), (Owner, ZERO), (ActiveId, ZERO)], 'D_S'))
            # debug
            transitions.append((None, 'D_BUSY', UnblockSMsgP[c][d][a], z3p.Eq(Sharers[c], ONE), [(Owner, ZERO), (ActiveId, ZERO)], 'D_S'))

            #  foreach c2 in CacheIDType (!= c2 c) {
            #      D_BUSY on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> S;
            #  }
            for c2 in range(num_caches):
                if c2 != c:
                    transitions.append((None, 'D_BUSY', DataMsgC2CP[c2][c][d][a], TRUE, [(DataBlk, DataMsgC2CP[c2][c][d][a]), (ActiveId, ZERO)], 'D_S'))
            #  // Transitions from BUSY_DATA
            #  D_BUSY_DATA on UnblockSMsg'[c][d][a] { Sharers[c] := true; } -> D_BUSY;
            #  foreach c2 in CacheIDType (!= c2 c) {
            #      D_BUSY_DATA on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> D_BUSY;
            #  }
            transitions.append((None, 'D_BUSY_DATA', UnblockSMsgP[c][d][a], z3p.Eq(Sharers[c], ZERO), [(Sharers[c], ONE), (NumSharers, NumSharers + 1)], 'D_BUSY'))
            # debug
            transitions.append((None, 'D_BUSY_DATA', UnblockSMsgP[c][d][a], z3p.Eq(Sharers[c], ONE), [], 'D_BUSY'))

            for c2 in range(num_caches):
                if c2 != c:
                    transitions.append((None, 'D_BUSY_DATA', DataMsgC2CP[c2][c][d][a], TRUE, [(DataBlk, DataMsgC2CP[c2][c][d][a])], 'D_BUSY'))
            #  D_BUSY_DATA on WBMsg'[c][d][a] (v)
            #  if (= c ActiveId) { DataBlk := v; } -> D_PENDING_UNBLOCK_E;
            #  if (!= c ActiveId) { Sharers[c] := false; DataBlk := v; } ->
            #  send DataMsgD2C[ActiveId][d][a] (DataBlk, 0) -> D_BUSY;
            transitions.append((None, 'D_BUSY_DATA', WBMsgP[c][d][a], z3p.Eq(z3p.IntVal(c), ActiveId), [(DataBlk, WBMsgP[c][d][a])], 'D_PENDING_UNBLOCK_E'))
            target = 'D_BUSY2_{}'.format(c)
            transitions.append((None, 'D_BUSY_DATA', WBMsgP[c][d][a], z3p.And(z3p.Neq(z3p.IntVal(c), ActiveId), z3p.Eq(Sharers[c], ONE)), [(Sharers[c], ZERO), (NumSharers, NumSharers - 1), (DataBlk, WBMsgP[c][d][a])], target))
            # debug
            transitions.append((None, 'D_BUSY_DATA', WBMsgP[c][d][a], z3p.And(z3p.Neq(z3p.IntVal(c), ActiveId), z3p.Eq(Sharers[c], ZERO)), [(DataBlk, WBMsgP[c][d][a])], target))

            # for c2 in range(num_caches):
            #     transitions.append((None, 'D_BUSY_DATA2', DataMsgD2C[c2][a], [DataBlk, ZERO], z3p.Eq(z3p.IntVal(c2), ActiveId), [], 'D_BUSY'))
            #  // Transitions from PENDING_UNBLOCK_E
            #  D_PENDING_UNBLOCK_E on UnblockEMsg'[c][d][a] { Sharers[c] := false; Owner := undef; } -> D_I;
            target = 'D_PENDING_UNBLOCK_E_{}'.format(c)
            transitions.append((None, 'D_PENDING_UNBLOCK_E', UnblockEMsgP[c][d][a], z3p.Eq(Sharers[c], ONE), [(Sharers[c], ZERO), (Owner, ZERO), (NumSharers, NumSharers - 1)], target))
            #debug
            transitions.append((None, 'D_PENDING_UNBLOCK_E', UnblockEMsgP[c][d][a], z3p.Eq(Sharers[c], ZERO), [], target))

            transitions.append((None, target, WBAckMsg[c][d][a], [ZERO], TRUE, [], 'D_I'))
            locations.append(target)

    return automaton.SymbolicAutomaton('directory',
                                       locations,
                                       'D_I',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=input_channels,
                                       output_channels=output_channels,
                                       variable_ranges=variable_ranges)

def request_channel(address_id, cache_id, directory_id, num_values):
    name = 'request_channel_{}_{}_{}'.format(address_id, cache_id, directory_id)
    inputs = []
    outputs = []
    suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
    for port in ['GetXMsg', 'GetSMsg']:
        inputs.append(util.new_variable(port + suffix, 'unit'))
        outputs.append(util.new_variable(port + 'P' + suffix, 'unit'))
    inputs.append(z3p.Int('WBMsg' + suffix))
    outputs.append(z3p.Int('WBMsgP' + suffix))
    data_ranges = {z3p.Int('WBMsg' + suffix): (0, num_values - 1)}
    return lossless_non_duplicating_blocking_capacity_one_channel(name, inputs, outputs, data_ranges, {})


def response_channel(address_id, cache_id, directory_id, num_values, num_caches):
    name = 'response_channel_{}_{}_{}'.format(cache_id, address_id, directory_id)
    inputs = []
    outputs = []
    data_ranges = {}
    number_of_fields = {}
    suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
    wbackmsg = 'WBAckMsg'
    wbackmsg_input = util.new_variable(wbackmsg + suffix, 'unit')
    wbackmsg_output = util.new_variable(wbackmsg + 'P' + suffix, 'unit')
    inputs.append(wbackmsg_input)
    outputs.append(wbackmsg_output)

    locations = ['empty']

    transitions = []

    full_state = 'WBAckMsg_full'
    locations.append(full_state)
    transitions.append((None, 'empty', wbackmsg_input, TRUE, [], full_state))
    transitions.append((None, full_state, wbackmsg_output, [ZERO], TRUE, [], 'empty'))

    variable_ranges = {}

    variable_data = z3p.Int('data_{}'.format(name))
    variable_num_caches = z3p.Int('num_caches_{}'.format(name))
    variable_ranges[variable_data] = (0, num_values - 1)
    variable_ranges[variable_num_caches] = (0, num_caches)

    datamsgd2c = 'DataMsgD2C'
    datamsgd2c_input = z3p.Array(datamsgd2c + suffix, z3p.IntSort(), z3p.IntSort())
    datamsgd2c_output = z3p.Array(datamsgd2c + 'P' + suffix, z3p.IntSort(), z3p.IntSort())
    inputs.append(datamsgd2c_input)
    outputs.append(datamsgd2c_output)

    full_state = 'DataMsgD2C_full'
    locations.append(full_state)
    transitions.append((None, 'empty', datamsgd2c_input, TRUE, [(variable_data, datamsgd2c_input[0]), (variable_num_caches, datamsgd2c_input[1])], full_state))
    transitions.append((None, full_state, datamsgd2c_output, [variable_data, variable_num_caches], TRUE, [(variable_data, ZERO), (variable_num_caches, ZERO)], 'empty'))

    datamsgc2c = 'DataMsgC2C'
    invackmsg = 'InvAckMsg'
    for other_cache_id in range(num_caches):
        if other_cache_id != cache_id:
            suffix2 = '_{}{}'.format(other_cache_id, suffix)
            datamsgc2c_input = z3p.Int(datamsgc2c + suffix2)
            datamsgc2c_output = z3p.Int(datamsgc2c + 'P' + suffix2)
            inputs.append(datamsgc2c_input)
            outputs.append(datamsgc2c_output)

            full_state = 'DataMsgC2C_full_{}'.format(other_cache_id)
            locations.append(full_state)
            transitions.append((None, 'empty', datamsgc2c_input, TRUE, [(variable_data, datamsgc2c_input)], full_state))
            transitions.append((None, full_state, datamsgc2c_output, [variable_data], TRUE, [(variable_data, ZERO)], 'empty'))

            invackmsg_input = util.new_variable(invackmsg + suffix2, 'unit')
            invackmsg_output = util.new_variable(invackmsg + 'P' + suffix2, 'unit')
            inputs.append(invackmsg_input)
            outputs.append(invackmsg_output)

            full_state = 'InvAckMsg_full_{}'.format(other_cache_id)
            locations.append(full_state)
            transitions.append((None, 'empty', invackmsg_input, TRUE, [], full_state))
            transitions.append((None, full_state, invackmsg_output, [ZERO], TRUE, [], 'empty'))
    return automaton.SymbolicAutomaton(name,
                                       locations,
                                       'empty',
                                       transitions,
                                       variables=[variable_data, variable_num_caches],
                                       initial_values=[ZERO, ZERO],
                                       input_channels=inputs,
                                       output_channels=outputs,
                                       variable_ranges=variable_ranges)


def unblock_channel(address_id, directory_id, num_caches):
    name = 'unblock_channel_{}_{}'.format(address_id, directory_id)
    inputs = []
    outputs = []
    transitions = []
    locations = ['empty']
    for cache_id in range(num_caches):
        suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
        for port in ['UnblockEMsg', 'UnblockSMsg']:
            input_channel_name = port + suffix
            input_channel = util.new_variable(input_channel_name, 'unit')
            inputs.append(input_channel)
            output_channel = util.new_variable(port + 'P' + suffix, 'unit')
            outputs.append(output_channel)
            full_state = 'full_{}'.format(input_channel_name)
            locations.append(full_state)
            transitions.append((None, 'empty', input_channel, TRUE, [], full_state))
            transitions.append((None, full_state, output_channel, [ZERO], TRUE, [], 'empty'))
    return automaton.SymbolicAutomaton('{}_channel'.format(name),
                                       locations,
                                       'empty',
                                       transitions,
                                       variables=[],
                                       initial_values=[],
                                       input_channels=inputs,
                                       output_channels=outputs)


def forward_channel(cache_id, directory_id, address_id, num_caches):
    name = 'forward_channel_{}_{}_{}'.format(cache_id, directory_id, address_id)
    variable = z3p.Int('forward_channel_{}'.format(cache_id))
    variables = [variable]
    initial_values = [ZERO]
    variable_ranges = {variable: (0, num_caches - 1)}
    suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
    inputs = []
    outputs = []

    locations = ['empty']

    fwd_get_smsg_input = z3p.Int('FwdGetSMsg' + suffix)
    inputs.append(fwd_get_smsg_input)
    fwd_get_xmsg_input = z3p.Int('FwdGetXMsg' + suffix)
    inputs.append(fwd_get_xmsg_input)
    fwd_get_smsg_output = z3p.Int('FwdGetSMsgP' + suffix)
    outputs.append(fwd_get_smsg_output)
    fwd_get_xmsg_output = z3p.Int('FwdGetXMsgP' + suffix)
    outputs.append(fwd_get_xmsg_output)

    transitions = []
    for i, (input_channel, output_channel) in enumerate(zip(inputs, outputs)):
        input_channel_name = input_channel.decl().name()
        input_transition_name = 't_input_{}'.format(input_channel_name)
        output_transition_name = 't_output_{}'.format(input_channel_name)
        full_state = 'full_{}'.format(input_channel_name)
        transitions.append((input_transition_name, 'empty', input_channel, TRUE, [(variable, input_channel)], full_state))
        locations.append(full_state)
        transitions.append((output_transition_name, full_state, output_channel, [variable], TRUE, [(variable, ZERO)], 'empty'))

    return automaton.SymbolicAutomaton('{}_channel'.format(name),
                                       locations,
                                       'empty',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=inputs,
                                       output_channels=outputs,
                                       variable_ranges=variable_ranges)




def environment(cache_id, directory_id, address_id, num_values):
    transitions = []
    variables = []
    variable_ranges = {}
    PendingStore = z3p.Int('pending_store_environment_{}_{}_{}'.format(cache_id, directory_id, address_id))
    StoreResult = z3p.Int('store_result_{}_{}_{}'.format(cache_id, directory_id, address_id))

    variable_ranges[PendingStore] = (0, num_values - 1)
    variable_ranges[StoreResult] = (0, num_values - 1)
    variables = [PendingStore, StoreResult]

    initial_values = [ZERO, ZERO]

    locations = ['Env{}_Initial'.format(cache_id),
                 'Env{}_PendingLD'.format(cache_id),
                 'Env{}_PendingST'.format(cache_id),
                 'Env{}_PendingEV'.format(cache_id),
                 'Env{}_Error'.format(cache_id)]

    # input messages
    suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
    LDAckMsg = z3p.Int('LDAckMsg' + suffix)
    STAckMsg = z3p.Int('STAckMsg' + suffix)
    EVAckMsg = util.new_variable('EVAckMsg' + suffix, 'unit')
    input_channels = [LDAckMsg, STAckMsg, EVAckMsg]
    # input_channels = [LDAckMsg]

    LDMsg = util.new_variable('LDMsg' + suffix, 'unit')
    STMsg = z3p.Int('STMsg' + suffix)
    EVMsg = util.new_variable('EVMsg' + suffix, 'unit')
    output_channels = [LDMsg, STMsg, EVMsg]
    # output_channels = [LDMsg]

    transitions = []
    # // Load flow
    # Env_Initial send LDMsg[c][d][a] {} -> Env_PendingLD;
    # Env_PendingLD on LDAckMsg[c][d][a] (v) {} -> Env_Initial;
    transitions.append((None, 'Env{}_Initial'.format(cache_id), LDMsg, [ZERO], TRUE, [], 'Env{}_PendingLD'.format(cache_id)))
    transitions.append((None, 'Env{}_PendingLD'.format(cache_id), LDAckMsg, TRUE, [(PendingStore, LDAckMsg)], 'Env{}_Initial'.format(cache_id)))

    # // Store flow
    # foreach v in ValueType
    # Env_Initial send STMsg[c][d][a] (v) { PendingStore := v; } -> Env_PendingST;
    # Env_PendingST on STAckMsg[c][d][a]
    # if (v = PendingStore) {} -> Env_Initial;
    # if (v != PendingStore) {} -> Env_Error;
    for v in range(num_values):
        transitions.append((None, 'Env{}_Initial'.format(cache_id), STMsg, [z3p.IntVal(v)], TRUE, [(PendingStore, z3p.IntVal(v))], 'Env{}_PendingST'.format(cache_id)))
        transitions.append((None, 'Env{}_PendingST'.format(cache_id), STAckMsg, TRUE, [(StoreResult, STAckMsg)], 'Env{}_PendingST2'.format(cache_id)))
        transitions.append((None, 'Env{}_PendingST2'.format(cache_id), z3p.Eq(StoreResult, PendingStore), [(StoreResult, ZERO), (PendingStore, ZERO)], 'Env{}_Initial'.format(cache_id)))
        transitions.append((None, 'Env{}_PendingST2'.format(cache_id), z3p.Neq(StoreResult, PendingStore), [(StoreResult, ZERO), (PendingStore, ZERO)], 'Env{}_Error'.format(cache_id)))

    # // Evict flow
    # Env_Initial send EVMsg[c][d][a] {} -> Env_PendingEV;
    # Env_PendingEV on EVAckMsg[c][d][a] {} -> Env_Initial;
    transitions.append((None, 'Env{}_Initial'.format(cache_id), EVMsg, [ZERO], TRUE, [], 'Env{}_PendingEV'.format(cache_id)))
    transitions.append((None, 'Env{}_PendingEV'.format(cache_id), EVAckMsg, TRUE, [], 'Env{}_Initial'.format(cache_id)))

    locations.extend(['Env{}_PendingST2'.format(cache_id)])

    return automaton.SymbolicAutomaton('enviroment_{}_{}_{}'.format(cache_id, directory_id, address_id),
                                       locations,
                                       'Env{}_Initial'.format(cache_id),
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=input_channels,
                                       output_channels=output_channels,
                                       variable_ranges=variable_ranges)


automata = ([forward_channel(cache_id, directory_id, address_id, NUM_CACHES) for address_id in range(NUM_ADDRESSES) for cache_id in range(NUM_CACHES) for directory_id in range(NUM_DIRECTORIES)]+
            [directory(directory_id, NUM_CACHES, NUM_VALUES, NUM_ADDRESSES) for directory_id in range(NUM_DIRECTORIES)] +
            [cache(cache_id, NUM_CACHES, NUM_VALUES, NUM_ADDRESSES, NUM_DIRECTORIES) for cache_id in range(NUM_CACHES)] +
            [request_channel(address_id, cache_id, directory_id, NUM_VALUES) for address_id in range(NUM_ADDRESSES) for cache_id in range(NUM_CACHES) for directory_id in range(NUM_DIRECTORIES)] +
            [response_channel(address_id, cache_id, directory_id, NUM_VALUES, NUM_CACHES) for address_id in range(NUM_ADDRESSES) for cache_id in range(NUM_CACHES) for directory_id in range(NUM_DIRECTORIES)] +
            [unblock_channel(address_id, directory_id, NUM_CACHES) for address_id in range(NUM_ADDRESSES) for directory_id in range(NUM_DIRECTORIES)] +
            [environment(cache_id, directory_id, address_id, NUM_VALUES) for address_id in range(NUM_ADDRESSES) for cache_id in range(NUM_CACHES) for directory_id in range(NUM_DIRECTORIES)])
