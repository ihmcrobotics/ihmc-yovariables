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

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Base class for parameter readers.
 * <p>
 * A parameter reader is used to initialize parameters from some external source, for instance a XML
 * file when using {@link XmlParameterReader}.
 * </p>
 *
 * @author Jesper Smith
 */
public abstract class AbstractParameterReader
{
   /**
    * Recurses starting from the given registry, finds all parameters, and initializes them using the
    * {@link #getValues()} map.
    * <p>
    * If {@link #getValues()} does not cover all parameters, the missed parameters are initialized to
    * default, see {@link YoParameter#loadDefault()}.
    * </p>
    *
    * @param registry with parameters that need to be loaded.
    */
   public void readParametersInRegistry(YoRegistry registry)
   {
      readParametersInRegistry(registry, new HashSet<>(), new HashSet<>());
   }

   /**
    * Recurses starting from the given registry, finds all parameters, and initializes them using the
    * {@link #getValues()} map.
    * <p>
    * If {@link #getValues()} does not cover all parameters, the missed parameters are initialized to
    * default, see {@link YoParameter#loadDefault()}.
    * </p>
    *
    * @param registry                  with parameters that need to be loaded.
    * @param defaultParametersToPack   will be set to contain all parameter names (incl. namespace)
    *                                  that use their default value.
    * @param unmatchedParametersToPack will be set to contain all parameter names (incl. namespace)
    *                                  that exist in the loader but have no matching parameter in the
    *                                  registry.
    */
   public void readParametersInRegistry(YoRegistry registry, Set<String> defaultParametersToPack, Set<String> unmatchedParametersToPack)
   {
      defaultParametersToPack.clear();
      unmatchedParametersToPack.clear();

      List<YoParameter> parameters = registry.subtreeParameters();
      Map<String, ParameterData> localMap = new HashMap<>(getValues());

      for (int i = 0; i < parameters.size(); i++)
      {
         YoParameter parameter = parameters.get(i);

         YoNamespace relativeNamespace = getRelativeNamespace(parameter.getNamespace(), registry);
         String fullName = relativeNamespace + "." + parameter.getName();
         ParameterData data = localMap.remove(fullName);

         if (data != null)
         {
            data.setParameterFromThis(parameter);
         }
         else
         {
            parameter.loadDefault();
            defaultParametersToPack.add(fullName);
         }
      }

      unmatchedParametersToPack.addAll(localMap.keySet());
   }

   /**
    * Returns a map from parameter full-name to parameter's initial values that is to be used in
    * {@link #readParametersInRegistry(YoRegistry, Set, Set)} for initializing the parameters.
    * 
    * @return the map containing the parameters' initial values.
    */
   protected abstract Map<String, ParameterData> getValues();

   static YoNamespace getRelativeNamespace(YoNamespace parameterNamespace, YoRegistry registry)
   {
      YoNamespace registryNamespace = registry.getNamespace();
      if (registryNamespace.isRoot())
         return parameterNamespace;
      else
         return parameterNamespace.removeStart(registry.getNamespace().removeEnd(1));
   }
}
