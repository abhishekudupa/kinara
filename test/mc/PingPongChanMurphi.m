CONST
    NUMCLIENTS : 2;
    NUMVALUES : 2;
    C2SCHANSIZE : 2;
    S2CCHANSIZE : 1;

TYPE
    ClientIDType : ScalarSet(NUMCLIENTS);
    ValueType : 0..NUMVALUES-1;
    MType : enum { DataMsg, AckMsg };
    MsgType : record
        mtype : MType;
        client : ClientIDType;
        value : ValueType;
    end;

    ClientStateType : enum { CInit, CRecv, CDecide, CError };
    ServerStateType : enum { SInit, SSend };

VAR
    C2SChan : record
        MsgBuffer : multiset[C2SCHANSIZE] of MsgType;
    end;

    S2CChan : array [ClientIDType] of record
        MsgCount : 0..S2CCHANSIZE;
        MsgBuffer : array[0..S2CCHANSIZE-1] of MsgType
    end;

    Client : array [ClientIDType] of record
        Count : ValueType;
        LastMsg : ValueType;
        State : ClientStateType;
    end;

    Server : record
        LastMsg : ValueType;
        LastReq : ClientIDType;
        State : ServerStateType;
    end;

Procedure PopS2CChan(id : ClientIDType);
begin
    if (S2CCHANSIZE = 1 | S2CChan[id].MsgCount = 1) then
        S2CChan[id].MsgCount := S2CChan[id].MsgCount - 1;
        undefine S2CChan[id].MsgBuffer[0];
    else
        for i := 0 to S2CChan[id].MsgCount-2 do
            S2CChan[id].MsgBuffer[i] := S2CChan[id].MsgBuffer[i+1];
        endfor;
        undefine S2CChan[id].MsgBuffer[S2CChan[id].MsgCount-1];
        S2CChan[id].MsgCount := S2CChan[id].MsgCount - 1;
    endif;
end;

Ruleset clientid : ClientIDType do
Rule "Send Message"
    (Client[clientid].State = CInit) &
    (Multisetcount(i:C2SChan.MsgBuffer, true) < C2SCHANSIZE) ==>
var
    OutMsg : MsgType;
begin
    Client[clientid].State := CRecv;
    OutMsg.mtype := DataMsg;
    OutMsg.client := clientid;
    OutMsg.value := Client[clientid].Count;
    Multisetadd(OutMsg, C2SChan.MsgBuffer);
endrule;

Rule "Recv Response"
    (Client[clientid].State = CRecv) &
    (S2CChan[clientid].MsgCount > 0) &
    (S2CChan[clientid].MsgBuffer[0].client = clientid) &
    (S2CChan[clientid].MsgBuffer[0].mtype = AckMsg) ==>
begin
    Client[clientid].State := CDecide;
    Client[clientid].LastMsg := S2CChan[clientid].MsgBuffer[0].value;
    PopS2CChan(clientid);
endrule;

Rule "Decide Response Good"
    (Client[clientid].State = CDecide) &
    (Client[clientid].LastMsg = Client[clientid].Count) ==>
begin
    Client[clientid].State := CInit;
    Client[clientid].Count := (Client[clientid].Count + 1) % NUMVALUES;
    undefine Client[clientid].LastMsg;
end;

Rule "Decide Response Bad"
    (Client[clientid].State = CDecide) &
    (!(Client[clientid].LastMsg = Client[clientid].Count)) ==>
begin
    Client[clientid].State := CError;
end;

endruleset;

Choose msindex : C2SChan.MsgBuffer Do
Rule "Server Receive"
    (Server.State = SInit) ==>
begin
    Server.LastReq := C2SChan.MsgBuffer[msindex].client;
    Server.LastMsg := C2SChan.MsgBuffer[msindex].value;
    Server.State := SSend;
    Multisetremove(msindex, C2SChan.MsgBuffer);
end;
endchoose;

Rule "Server Respond"
    (Server.State = SSend) &
    (S2CChan[Server.LastReq].MsgCount < S2CCHANSIZE) ==>
var
    OutMsg : MsgType;
begin
    OutMsg.client := Server.LastReq;
    OutMsg.mtype := AckMsg;
    OutMsg.value := Server.LastMsg;
    S2CChan[Server.LastReq].MsgBuffer[S2CChan[Server.LastReq].MsgCount] := OutMsg;
    S2CChan[Server.LastReq].MsgCount := S2CChan[Server.LastReq].MsgCount + 1;
    Server.State := SInit;
    undefine Server.LastReq;
    undefine Server.LastMsg;
end;

StartState
begin
    for clientid : ClientIDType do
        Client[clientid].Count := 0;
        undefine Client[clientid].LastMsg;
        Client[clientid].State := CInit;

        S2CChan[clientid].MsgCount := 0;
        undefine S2CChan[clientid].MsgBuffer;
    endfor;

    undefine C2SChan;
    undefine Server.LastReq;
    undefine Server.LastMsg;
    Server.State := SInit;
end;

invariant "No error"
    forall clientid : ClientIDType do
        (!(Client[clientid].State = CError))
    end;
