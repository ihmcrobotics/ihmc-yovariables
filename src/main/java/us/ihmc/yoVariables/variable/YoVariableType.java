/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.variable;

/**
 * Enumerates the primitives implemented as {@code YoVariable} and {@code YoParameter}.
 */
public enum YoVariableType
{
   /** Counterpart of {@code Double}: {@code YoDouble} and {@code DoubleParameter}. */
   DOUBLE,
   /** Counterpart of {@code Boolean}: {@code YoBoolean} and {@code BooleanParameter}. */
   BOOLEAN,
   /** Counterpart of {@code Enum}: {@code YoEnum} and {@code EnumParameter}. */
   ENUM,
   /** Counterpart of {@code Integer}: {@code YoInteger} and {@code IntegerParameter}. */
   INTEGER,
   /** Counterpart of {@code Long}: {@code YoLong} and {@code LongParameter}. */
   LONG;
}
