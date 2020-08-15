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
package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code Point2DBasics} implementation which coordinates {@code x}, {@code y} are backed with
 * {@code YoDouble}s.
 */
public class YoPoint2D extends YoTuple2D implements Point2DBasics
{
   /**
    * Creates a new yo point using the given variables.
    *
    * @param xVariable the x-coordinate variable.
    * @param yVariable the y-coordinate variable.
    */
   public YoPoint2D(YoDouble xVariable, YoDouble yVariable)
   {
      super(xVariable, yVariable);
   }

   /**
    * Creates a new yo point, initializes its coordinates to zero, and registers variables to
    * {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoPoint2D(String namePrefix, YoRegistry registry)
   {
      super(namePrefix, registry);
   }

   /**
    * Creates a new yo point, initializes its coordinates to zero, and registers variables to
    * {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoPoint2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }
}