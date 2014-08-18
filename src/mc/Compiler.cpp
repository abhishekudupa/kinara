// Compiler.cpp --- 
// 
// Filename: Compiler.cpp
// Author: Abhishek Udupa
// Created: Mon Aug 18 12:02:24 2014 (-0400)
// 
// 
// Copyright (c) 2013, Abhishek Udupa, University of Pennsylvania
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by The University of Pennsylvania
// 4. Neither the name of the University of Pennsylvania nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// 

// Code:

#include <boost/lexical_cast.hpp>

#include "../uflts/LabelledTS.hpp"

#include "Compiler.hpp"
#include "StateVec.hpp"


namespace ESMC {
    namespace MC {

        using namespace ESMC::LTS;
        
        RValueInterpreter::RValueInterpreter(const ExprTypeRef& Type)
            : Type(Type), Size(Type->GetByteSize()), 
              IsScalar(Type->Is<Exprs::ExprScalarType>())
        {
            // Nothing here
        }

        RValueInterpreter::~RValueInterpreter()
        {
            // Nothing here
        }

        void RValueInterpreter::MakeInterpreter(const ExpT& Exp, const LabelledTS* TheLTS)
        {
            if (Exp->ExtensionData.Interps.RValInterp != nullptr) {
                return;
            }

            auto ExpAsVar = Exp->As<Exprs::VarExpression>();
            if (ExpAsVar != nullptr) {
                bool IsMsg = false;
                if (TheLTS->GetProductMsgName() == ExpAsVar->GetVarName() &&
                    TheLTS->GetUnifiedMType() == ExpAsVar->GetVarType()) {
                    IsMsg = true;
                    Exp->ExtensionData.Offset = 0;
                }
                if (ExpAsVar->ExtensionData.Offset == -1) {
                    throw InternalError((string)"Expected offset to be set on var expression.\n" + 
                                        "At: " + __FILE__ + ":" + to_string(__LINE__));
                }

                Exp->ExtensionData.Interps.LValInterp = 
                    new LValueInterpreter(Exp->GetType(), IsMsg, Exp->ExtensionData.Offset);
            }
            
            auto ExpAsConst = Exp->As<Exprs::ConstExpression>();
            if (ExpAsConst != nullptr) {
                Exp->ExtensionData.Interps.RValInterp =
                    new ConstantInterpreter(Exp);
            }

            auto ExpAsOp = Exp->As<Exprs::OpExpression>();
            if (ExpAsOp == nullptr) {
                throw InternalError((string)"Unsupported expression kind in compiler: " + 
                                    Exp->ToString() + "\nAt: " + __FILE__ + ":" + 
                                    to_string(__LINE__));
            }

            auto const& Children = ExpAsOp->GetChildren();
            for (auto const& Child : Children) {
                MakeInterpreter(Child, TheLTS);
            }

            auto OpCode = ExpAsOp->GetOpCode();
            switch (OpCode) {
                // TODO: Fix up interpreters for rest
            case LTSOps::OpIndex:
                if (Children[1]->ExtensionData.Interps.RValInterp->Is<ConstantInterpreter>()) {
                }
            }
        }
        
    } /* end namespace */
} /* end namespace */

// 
// Compiler.cpp ends here
