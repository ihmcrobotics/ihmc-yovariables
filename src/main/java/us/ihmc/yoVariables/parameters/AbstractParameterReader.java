/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC) Licensed under the Apache
 * License, Version 2.0 (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the License for the specific language governing permissions and limitations under
 * the License.
 */
package us.ihmc.yoVariables.parameters;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Base class for parameter readers
 *
 * @author Jesper Smith
 */
public abstract class AbstractParameterReader
{
   /**
    * Read all parameters registered to the registry and all its children. If a parameter cannot be
    * found in the parameter store, it is initialized to its default.
    *
    * @param registry with parameters that need to be loaded.
    */
   public void readParametersInRegistry(YoRegistry registry)
   {
      readParametersInRegistry(registry, new HashSet<>(), new HashSet<>());
   }

   /**
    * Read all parameters registered to the registry and all its children. If a parameter cannot be
    * found in the parameter store, it is initialized to its default.
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

      List<YoParameter<?>> parameters = registry.subtreeParameters();
      Map<String, ParameterData> localMap = new HashMap<>(getValues());

      for (int i = 0; i < parameters.size(); i++)
      {
         YoParameter<?> parameter = parameters.get(i);

         NameSpace relativeNamespace = getRelativeNamespace(parameter.getNameSpace(), registry);
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

   protected abstract Map<String, ParameterData> getValues();

   static NameSpace getRelativeNamespace(NameSpace parameterNamespace, YoRegistry registry)
   {
      NameSpace registryNamespace = registry.getNameSpace();
      if (registryNamespace.isRoot())
         return parameterNamespace;
      else
         return parameterNamespace.removeStart(registry.getNameSpace().removeEnd(1));
   }
}
