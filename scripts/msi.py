import automaton
import util
import z3p


def cache(cache_id, num_caches=2, num_values=2, num_addresses=2, num_dirs=1):
    transitions = []
    variables = []
    variable_ranges = {}
    DataBlk = z3p.Int('DataBlk')
    AckCounter = z3p.Int('AckCounter')
    PendingWrite = z3p.Int('PendingWrite')
    FwdToCache = z3p.Int('FwdToCache')
    NumAcksTemp = z3p.Int('NumAcksTemp')
    variable_ranges = {DataBlk: (0, num_values - 1),
                       AckCounter: (-num_caches, num_caches),
                       PendingWrite: (0, num_values - 1),
                       FwdToCache: (0, num_caches - 1),
                       NumAcksTemp: (0, num_caches)}
    variables = [DataBlk, AckCounter, PendingWrite, FwdToCache, NumAcksTemp]
    initial_values = [z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0)]
    # ??? initial_values

    input_channels = []
    # input messages
    LDMsg = {}
    STMsg = {}
    EVMsg = {}
    FwdGetXMsgP = {}
    FwdGetSMsgP = {}
    InvAckMsg = {}
    InvAckMsgP = {}
    DataMsgD2CP = {}
    WBAckMsgP = {}
    DataMsgC2CP = {}
    for d in range(num_dirs):
        LDMsg[d] = {}
        STMsg[d] = {}
        EVMsg[d] = {}
        FwdGetXMsgP[d] = {}
        FwdGetSMsgP[d] = {}
        InvAckMsgP[d] = {}
        DataMsgD2CP[d] = {}
        DataMsgC2CP[d] = {}
        WBAckMsgP[d] = {}
        for a in range(num_addresses):
            suffix = '_{}_{}_{}'.format(cache_id, d, a)
            LDMsg[d][a] = util.new_variable('LDMsg' + suffix, 'unit')
            input_channels.append(LDMsg[d][a])
            STMsg[d][a] = z3p.Int('STMsg' + suffix)
            input_channels.append(STMsg[d][a])
            EVMsg[d][a] = util.new_variable('EVMsg' + suffix, 'unit')
            input_channels.append(EVMsg[d][a])
            FwdGetXMsgP[d][a] = z3p.Int('FwdGetXMsgP' + suffix)
            input_channels.append(FwdGetXMsgP[d][a])
            FwdGetSMsgP[d][a] = z3p.Int('FwdGetSMsgP' + suffix)
            input_channels.append(FwdGetSMsgP[d][a])
            DataMsgD2CP[d][a] = z3p.Array('DataMsgD2CP' + suffix, z3p.IntSort(), z3p.IntSort())
            input_channels.append(DataMsgD2CP[d][a])
            WBAckMsgP[d][a] = z3p.Int('WBAckMsgP' + suffix)
            input_channels.append(WBAckMsgP[d][a])
            InvAckMsgP[d][a] = {}
            DataMsgC2CP[d][a] = {}
            for other_c in range(num_caches):
                if other_c != cache_id:
                    suffix2 = '{}{}'.format(other_c, suffix)
                    InvAckMsgP[d][a][other_c] = z3p.Int('InvAckMsgP' + suffix2)
                    input_channels.append(InvAckMsgP[d][a][other_c])
                    DataMsgC2CP[d][a][other_c] = z3p.Int('DataMsgC2CP' + suffix2)
                    input_channels.append(DataMsgC2CP[d][a][other_c])

    output_channels = []
    # output messages
    LDAckMsg = {}
    STAckMsg = {}
    EVAckMsg = {}
    UnblockSMsg = {}
    UnblockEMsg = {}
    InvAckMsg = {}
    GetXMsg = {}
    GetSMsg = {}
    WBMsg = {}
    DataMsgC2C = {}
    for d in range(num_dirs):
        LDAckMsg[d] = {}
        STAckMsg[d] = {}
        EVAckMsg[d] = {}
        UnblockSMsg[d] = {}
        UnblockEMsg[d] = {}
        InvAckMsg[d] = {}
        GetXMsg[d] = {}
        GetSMsg[d] = {}
        WBMsg[d] = {}
        DataMsgC2C[d] = {}
        for a in range(num_addresses):
            suffix = '_{}_{}_{}'.format(cache_id, d, a)
            LDAckMsg[d][a] = util.new_variable('LDAckMsg' + suffix, 'unit')
            output_channels.append(LDAckMsg[d][a])
            STAckMsg[d][a] = z3p.Int('STAckMsg' + suffix)
            output_channels.append(STAckMsg[d][a])
            EVAckMsg[d][a] = util.new_variable('EVAckMsg' + suffix, 'unit')
            output_channels.append(EVAckMsg[d][a])
            UnblockSMsg[d][a] = util.new_variable('UnblockSMsg' + suffix, 'unit')
            output_channels.append(UnblockSMsg[d][a])
            UnblockEMsg[d][a] = util.new_variable('UnblockEMsg' + suffix, 'unit')
            output_channels.append(UnblockEMsg[d][a])

            GetXMsg[d][a] = z3p.Int('GetXMsg' + suffix)
            output_channels.append(GetXMsg[d][a])
            GetSMsg[d][a] = z3p.Int('GetSMsg' + suffix)
            output_channels.append(GetSMsg[d][a])
            WBMsg[d][a] = z3p.Int('WBMsg' + suffix)
            output_channels.append(WBMsg[d][a])

            InvAckMsg[d][a] = {}
            DataMsgC2C[d][a] = {}
            for other_c in range(num_caches):
                if other_c != cache_id:
                    suffix2 = '{}_{}_{}_{}'.format(cache_id, other_c, d, a)
                    InvAckMsg[d][a][other_c] = util.new_variable('InvAckMsg' + suffix2, 'unit')
                    output_channels.append(InvAckMsg[d][a][other_c])
                    DataMsgC2C[d][a][other_c] = z3p.Int('DataMsgC2C' + suffix2)
                    output_channels.append(DataMsgC2C[d][a][other_c])

    c = cache_id

    for d in range(num_dirs):
        for a in range(num_addresses):
            # // Transitions from I for Ext events
            # C_I on LDMsg[c][d][a] {} -> send GetSMsg[c][d][a] {} -> C_IS;
            # C_I on STMsg[c][d][a] (v) { PendingWrite := v; } ->
            # send GetXMsg[c][d][a] {} -> C_IM;
            # C_I on EVMsg[c][d][a] {} -> send EVAckMsg[c][d][a] -> C_I;
            transitions.append((None, 'C_I', LDMsg[d][a], z3p.BoolVal(True), [], 'C_I2'))
            transitions.append((None, 'C_I2', GetSMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_IS'))
            transitions.append((None, 'C_I', STMsg[d][a], z3p.BoolVal(True), [(PendingWrite, STMsg[d][a])], 'C_I3'))
            transitions.append((None, 'C_I3', GetXMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_IM'))
            transitions.append((None, 'C_I', EVMsg[d][a], z3p.BoolVal(True), [], 'C_I4'))
            transitions.append((None, 'C_I4', EVAckMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_I'))

            # // Transitions for I on Fwd events
            # C_I on FwdGetSMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] { FwdToCache := undef; } -> C_I;
            transitions.append((None, 'C_I', FwdGetSMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetSMsgP[d][a])], 'C_I5'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_I5', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.BoolVal(0))], 'C_I'))


            # // Transitions from S on Ext Events
            # C_S on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) -> C_S;
            transitions.append((None, 'C_S', LDMsg[d][a], z3p.BoolVal(True), [], 'C_S2'))
            transitions.append((None, 'C_S2', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_S'))

            # C_S on STMsg[c][d][a] (v) { PendingWrite := v; } ->
            # send GetXMsg[c][d][a] {} -> C_SM;
            transitions.append((None, 'C_S', STMsg[d][a], z3p.BoolVal(True), [(PendingWrite, STMsg[d][a])], 'C_S3'))
            transitions.append((None, 'C_S3', GetXMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_SM'))

            # C_S on EVMsg[c][d][a] {} ->
            # send EVAckMsg[c][d][a] { DataBlk := undef; } -> C_I;
            transitions.append((None, 'C_S', EVMsg[d][a], z3p.BoolVal(True), [], 'C_S4'))
            transitions.append((None, 'C_S4', EVAckMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [(DataBlk, z3p.IntVal(0))], 'C_SM'))

            # // Transitions from S on Fwd events
            # C_S on FwdGetXMsg[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] { DataBlk := undef } -> C_I;
            transitions.append((None, 'C_S', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_S5'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_S5', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.BoolVal(0))], 'C_I'))

            # // Transitions from M on Ext events
            # C_M on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_M;
            transitions.append((None, 'C_M', LDMsg[d][a], z3p.BoolVal(True), [], 'C_M2'))
            transitions.append((None, 'C_M2', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))
            # C_M on STMsg[c][d][a] (v) { DataBlk := v; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            transitions.append((None, 'C_M', STMsg[d][a], z3p.BoolVal(True), [(DataBlk, STMsg[d][a])], 'C_M3'))
            transitions.append((None, 'C_M3', STAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))

            # C_M on EVMsg[c][d][a] {} -> send WBMsg[c][d][a] (DataBlk) {} -> C_II;
            transitions.append((None, 'C_M', EVMsg[d][a], z3p.BoolVal(True), [], 'C_M4'))
            transitions.append((None, 'C_M4', WBMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_II'))


            # // Transitions from M on Fwd events
            # C_M on FwdGetSMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send DataMsgC2C[c][FwdToCache][d][a] (DataBlk) { FwdToCache := undef; } -> C_S;
            transitions.append((None, 'C_M', FwdGetSMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetSMsgP[d][a])], 'C_M5'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_M5', DataMsgC2C[d][a][other_c], [DataBlk], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.IntVal(0))], 'C_S'))

            # C_M on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send DataMsgC2C[c][FwdToCache][d][a] (DataBlk)
            # { FwdToCache := undef; DataBlk := undef; } -> C_I;
            transitions.append((None, 'C_M', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_M6'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_M6', DataMsgC2C[d][a][other_c], [DataBlk], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.IntVal(0)), (DataBlk, z3p.IntVal(0))], 'C_I'))

            # // Transitions from C_IM on Rsp Events
            # C_IM on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {FwdToCache := undef; } -> C_IM;
            transitions.append((None, 'C_IM', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_IM2'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IM2', InvAckMsg[d][a][other_c], [DataBlk], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.IntVal(0))], 'C_IM'))
            # // Case when another cache gives me data
            # foreach c2 in CacheIDType (!= c2 c) {
            # C_IM on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> C_M;
            # }
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IM', DataMsgC2CP[d][a][other_c], z3p.BoolVal(True), [(DataBlk, DataMsgC2CP)], 'C_M'))

            # // Case when data comes from dir ==> Must wait for acks
            # C_IM on DataMsgD2C'[c][d][a] (v, NumAcks)
            # if (!= AckCounter NumAcks)
            # { AckCounter := (- AckCounter NumAcks); DataBlk := v; } -> C_SM;
            # if (= AckCounter NumAcks)
            # { AckCounter := 0; DataBlk := v; } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } -> C_M;
            transitions.append((None, 'C_IM', DataMsgD2CP[d][a], z3p.BoolVal(True), [(NumAcksTemp, DataMsgD2CP[d][a][1]), (DataBlk, DataMsgD2CP[d][a][0])], 'C_IM3'))
            transitions.append((None, 'C_IM3', z3p.Neq(AckCounter, NumAcksTemp), [(AckCounter, AckCounter - NumAcksTemp)], 'C_SM'))
            transitions.append((None, 'C_IM3', z3p.Eq(AckCounter, NumAcksTemp), [(AckCounter, z3p.BoolVal(0))], 'C_IM4'))
            transitions.append((None, 'C_IM4', UnblockEMsg[d][a], [z3p.IntVal(0)], [(DataBlk, PendingWrite), (PendingWrite, z3p.IntVal(0))], 'C_M'))

            # foreach c2 in CacheIDType (!= c2 c) {
            #     C_IM on InvAckMsg[c2][c][d][a] { AckCounter := (+ AckCounter 1); } -> C_IM;
            # }
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IM', InvAckMsg[d][a][other_c], z3p.BoolVal(True), [(AckCounter, AckCounter + 1)], 'C_IM'))

            # // Transitions from C_SM on Ext Events
            # C_SM on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_SM;
            transitions.append((None, 'C_SM', LDMsg[d][a], z3p.BoolVal(True), [], 'C_SM2'))
            transitions.append((None, 'C_SM2', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_SM'))

            # // Transitions from C_SM on Rsp Events
            # C_SM on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {} -> C_SM;
            transitions.append((None, 'C_SM', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_SM3'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_SM3', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [], 'C_SM'))


            # // Case where data comes from dir ==> Must wait for acks
            # C_SM on DataMsgD2C'[c][d][a] (v, NumAcks)

            transitions.append((None, 'C_SM', DataMsgD2CP[d][a], z3p.BoolVal(True), [(NumAcksTemp, DataMsgD2CP[d][a][1])], 'C_SM4'))

            # if (= NumAcks AckCounter) { AckCounter := 0; } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            # if (!= NumAcks AckCounter) { AckCounter := (- AckCounter NumAcks) } -> C_SM;
            transitions.append((None, 'C_SM4', UnblockEMsg[d][a], [z3p.IntVal(0)], z3p.Eq(NumAcksTemp, AckCounter), [(DataBlk, PendingWrite), (PendingWrite, z3p.IntVal(0))], 'C_SM5'))
            transitions.append((None, 'C_SM5', STAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))
            transitions.append((None, 'C_SM4', z3p.Neq(NumAcksTemp, AckCounter), [(AckCounter, AckCounter - NumAcksTemp)], 'C_SM'))

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
                if other_c != cache_id:
                    transitions.append((None, 'C_SM', InvAckMsgP[d][a][other_c], z3p.BoolVal(True), [], 'C_SM5'))
                    transitions.append((None, 'C_SM5', UnblockEMsg[d][a], [z3p.IntVal(0)], z3p.Eq(AckCounter, z3p.IntVal(-1)), [(AckCounter, z3p.IntVal(0)), (DataBlk, PendingWrite), (PendingWrite, z3p.IntVal(0))], 'C_SM6'))
                    transitions.append((None, 'C_SM6', STAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))
                    transitions.append((None, 'C_SM5', UnblockEMsg[d][a], [z3p.IntVal(0)], z3p.Neq(AckCounter, z3p.IntVal(-1)), [(AckCounter, AckCounter + 1), (DataBlk, PendingWrite), (PendingWrite, z3p.IntVal(0))], 'C_SM7'))
                    transitions.append((None, 'C_SM7', STAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))

            # // Transitions from IS on Ext Events
            # C_IS on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {} -> C_IS;
            transitions.append((None, 'C_IS', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_IS2'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IS2', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [], 'C_IS'))

            # // Transitions on IS on Rsp Events
            # C_IS on DataMsgD2C'[c][d][a] (v, _) { DataBlk := v; } ->
            # send UnblockSMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_S;
            transitions.append((None, 'C_IS', DataMsgD2CP[d][a], z3p.BoolVal(True), [(DataBlk, DataMsgD2CP[d][a][0])], 'C_IS2'))
            transitions.append((None, 'C_IS2', UnblockSMsg[d][a], z3p.BoolVal(True), [], 'C_IS3'))
            transitions.append((None, 'C_IS3', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_S'))

            # foreach c2 in CacheIDType (!= c2 c) {
            # C_IS on DataMsgC2C'[c2][c][d][a] (v) -> { DataBlk := v; } ->
            # send UnblockSMsg[c][d][a] -> send LDAckMsg[c][d][a] (DataBlk) -> C_S;
            # }
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IS', DataMsgC2CP[d][a][other_c], z3p.BoolVal(True), [(DataBlk, DataMsgC2CP[d][a][other_c])], 'C_IS4'))
                    transitions.append((None, 'C_IS4', UnblockSMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_IS5'))
                    transitions.append((None, 'C_IS5', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_S'))

            # // Transitions on II on Rsp Events
            # C_II on WBAckMsg'[c][d][a] -> C_I;
            transitions.append((None, 'C_II', WBAckMsgP[d][a], z3p.BoolVal(True), [], 'C_I'))

            # C_II on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] -> C_I;
            transitions.append((None, 'C_II', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_II2'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_II2', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(FwdToCache, other_c), [], 'C_I'))

            # C_II on FwdGetSMsg'[c][d][a] (_) -> I;
            # }
            transitions.append((None, 'C_II', FwdGetSMsgP[d][a], z3p.BoolVal(True), [], 'C_I'))


    locations = ['C_I', 'C_S', 'C_M', 'C_IM', 'C_SM', 'C_IS', 'C_II']
    locations.extend(['C_I2', 'C_I3', 'C_I4', 'C_I5'])
    locations.extend(['C_S2', 'C_S3', 'C_S4', 'C_S5'])
    locations.extend(['C_M2', 'C_M3', 'C_M4', 'C_M5', 'C_M6'])
    locations.extend(['C_IM2', 'C_IM3', 'C_IM4'])
    locations.extend(['C_SM2', 'C_SM3', 'C_SM4', 'C_SM5', 'C_SM6', 'C_SM7'])
    locations.extend(['C_IS2', 'C_IS3', 'C_IS4', 'C_IS5'])
    locations.extend(['C_II2'])


    return automaton.SymbolicAutomaton('cache{}'.format(cache_id),
                                       locations,
                                       'C_I',
                                       transitions,
                                       variables=[],
                                       initial_values=[],
                                       input_channels=input_channels,
                                       output_channels=output_channels,
                                       variable_ranges={})


def directory():
           # // Transitions from I
           #  D_I on GetXMsg'[c][d][a] { ActiveId := c; } ->
           #  send DataMsgD2C[c][d][a] (DataBlk, 0) -> D_BUSY;

           #  D_I on GetSMsg'[c][d][a] { ActiveId := c; } ->
           #  send DataMsgD2C[c][d][a] (DataBlk, 0) -> D_BUSY;

           #  // Transitions from S
           #  D_S on GetXMsg'[c][d][a] { ActiveId := c; } ->
           #  send DataMsgD2C[c][d][a] (DataBlk, NumSharers) {} ->
           #  foreach c2 in CacheIDType {
           #      if (Sharers[c2]) send FwdGetXMsg[c2][d][a] (c) {};
           #      if (not Sharers[c2]) pass {};
           #  } -> D_BUSY;

           #  D_S on GetSMsg'[c][d][a] { ActiveId := c; } ->
           #  send DataMsgD2C[c][d][a] (DataBlk, 0) {} -> D_BUSY;

           #  // Transitions from M
           #  D_M on GetXMsg'[c][d][a] { ActiveId := c; } ->
           #  send FwdGetXMsg[Owner][d][a] (ActiveId) {} -> D_BUSY;

           #  D_M on GetSMsg'[c][d][a] { ActiveId := c; } ->
           #  send FwdGetSMsg[Owner][d][a] (ActiveId) {} -> D_BUSY_DATA;

           #  D_M on WBMsg'[c][d][a] (v) { DataBlk := v; Sharers[c] := false; } ->
           #  send WBAckMsg[c][d][a] {} -> D_I;

           #  // Transitions from BUSY
           #  D_BUSY on WBMsg'[c][d][a] (v)
           #  if (= c ActiveId) { DataBlk := v; } -> D_PENDING_UNBLOCK_E;
           #  if (!= c ActiveId) { Sharers[c] := false; DataBlk := v; } ->
           #  send DataMsgD2C[ActiveId][d][a] (DataBlk, 0) -> D_BUSY;

           #  D_BUSY on UnblockEMsg'[c][d][a] { Sharers[c] := true; Owner := c; } -> D_M;

           #  D_BUSY on UnblockSMsg'[c][d][a] { Sharers[c] := true; Owner := undef; } -> D_S;

           #  foreach c2 in CacheIDType (!= c2 c) {
           #      D_BUSY on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> S;
           #  }

           #  // Transitions from BUSY_DATA
           #  D_BUSY_DATA on UnblockSMsg'[c][d][a] { Sharers[c] := true; } -> D_BUSY;
           #  foreach c2 in CacheIDType (!= c2 c) {
           #      D_BUSY_DATA on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> D_BUSY;
           #  }

           #  D_BUSY_DATA on WBMsg'[c][d][a] (v)
           #  if (= c ActiveId) { DataBlk := v; } -> D_PENDING_UNBLOCK_E;
           #  if (!= c ActiveId) { Sharers[c] := false; DataBlk := v; } ->
           #  send DataMsgD2C[ActiveId][d][a] (DataBlk, 0) -> D_BUSY;

           #  // Transitions from PENDING_UNBLOCK_E
           #  D_PENDING_UNBLOCK_E on UnblockEMsg'[c][d][a] { Sharers[c] := false; Owner := undef; } -> D_I;
    # locations = []
    # return automaton.SymbolicAutomaton('cache{}'.format(cache_id),
    #                                    locations,
    #                                    'D_I',
    #                                    transitions,
    #                                    variables=[],
    #                                    initial_values=[],
    #                                    input_channels=input_channels,
    #                                    output_channels=output_channels,
    #                                    variable_ranges={})
    pass
