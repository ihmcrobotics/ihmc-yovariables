/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
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
package us.ihmc.yoVariables.parameters;

import java.util.List;

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Base class for parameter writers.
 * <p>
 * A parameter writer is used to export parameters to some external destination, for instance a XML
 * file when using {@link XmlParameterWriter}.
 * </p>
 *
 * @author Jesper Smith
 */
public abstract class AbstractParameterWriter
{
   /**
    * Recurses starting from the given registry, finds all parameters, and calls for each
    * {@link #setValue(NameSpace, String, String, String, String, String, String)}.
    * 
    * @param registry with parameters that need to be exported.
    */
   public void addParameters(YoRegistry registry)
   {
      List<YoParameter> parameters = registry.subtreeParameters();

      for (int i = 0; i < parameters.size(); i++)
      {
         YoParameter parameter = parameters.get(i);

         NameSpace relativeNamespace = AbstractParameterReader.getRelativeNamespace(parameter.getNameSpace(), registry);

         String value = parameter.getValueAsString();
         String min = String.valueOf(parameter.getVariable().getLowerBound());
         String max = String.valueOf(parameter.getVariable().getUpperBound());
         setValue(relativeNamespace, parameter.getName(), parameter.getDescription(), parameter.getClass().getSimpleName(), value, min, max);

      }
   }

   /**
    * Exports the data for a parameter.
    */
   protected abstract void setValue(NameSpace namespace, String name, String description, String type, String value, String min, String max);
}
