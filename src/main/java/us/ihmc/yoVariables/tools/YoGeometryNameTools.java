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
package us.ihmc.yoVariables.tools;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;

/**
 * This class provides tools that helps standardizing name creation for classes implementing Euclid
 * types with {@code YoVariable}s such as {@link YoPoint2D} or {@link YoFrameQuaternion}.
 */
public class YoGeometryNameTools
{
   /**
    * Finds the largest prefix that is common to all the given {@code strings}.
    * 
    * @param strings the strings to find the common prefix of.
    * @return the common prefix or an empty {@code String} if no common prefix could be found.
    */
   public static String getCommonPrefix(String... strings)
   {
      return StringUtils.getCommonPrefix(strings);
   }

   /**
    * Finds the largest suffix that is common to all the given {@code strings}.
    * 
    * @param strings the strings to find the common suffix of.
    * @return the common suffix or an empty {@code String} if no common suffix could be found.
    */
   public static String getCommonSuffix(String... strings)
   {
      if (strings == null || strings.length == 0)
      {
         return "";
      }

      String[] reversedStrs = new String[strings.length];
      for (int i = 0; i < strings.length; i++)
         reversedStrs[i] = StringUtils.reverse(strings[i]);
      return StringUtils.reverse(StringUtils.getCommonPrefix(reversedStrs));
   }

   /**
    * Creates a variable name for a x-component given a prefix and a suffix.
    * 
    * @param namePrefix the name prefix.
    * @param nameSuffix the name suffix.
    * @return the name for a x-component.
    */
   public static String createXName(String namePrefix, String nameSuffix)
   {
      return assembleName(namePrefix, "x", nameSuffix);
   }

   /**
    * Creates a variable name for a y-component given a prefix and a suffix.
    * 
    * @param namePrefix the name prefix.
    * @param nameSuffix the name suffix.
    * @return the name for a y-component.
    */
   public static String createYName(String namePrefix, String nameSuffix)
   {
      return assembleName(namePrefix, "y", nameSuffix);
   }

   /**
    * Creates a variable name for a z-component given a prefix and a suffix.
    * 
    * @param namePrefix the name prefix.
    * @param nameSuffix the name suffix.
    * @return the name for a z-component.
    */
   public static String createZName(String namePrefix, String nameSuffix)
   {
      return assembleName(namePrefix, "z", nameSuffix);
   }

   /**
    * Creates a variable name for a x-component of a quaternion given a prefix and a suffix.
    * 
    * @param namePrefix the name prefix.
    * @param nameSuffix the name suffix.
    * @return the name for a x-component.
    */
   public static String createQxName(String namePrefix, String nameSuffix)
   {
      return assembleName(namePrefix, "qx", nameSuffix);
   }

   /**
    * Creates a variable name for a y-component of a quaternion given a prefix and a suffix.
    * 
    * @param namePrefix the name prefix.
    * @param nameSuffix the name suffix.
    * @return the name for a y-component.
    */
   public static String createQyName(String namePrefix, String nameSuffix)
   {
      return assembleName(namePrefix, "qy", nameSuffix);
   }

   /**
    * Creates a variable name for a z-component of a quaternion given a prefix and a suffix.
    * 
    * @param namePrefix the name prefix.
    * @param nameSuffix the name suffix.
    * @return the name for a z-component.
    */
   public static String createQzName(String namePrefix, String nameSuffix)
   {
      return assembleName(namePrefix, "qz", nameSuffix);
   }

   /**
    * Creates a variable name for a s-component of a quaternion given a prefix and a suffix.
    * 
    * @param namePrefix the name prefix.
    * @param nameSuffix the name suffix.
    * @return the name for a s-component.
    */
   public static String createQsName(String namePrefix, String nameSuffix)
   {
      return assembleName(namePrefix, "qs", nameSuffix);
   }

   /**
    * Concatenates the given sub-names into a single name while managing casing of the second to last
    * sub-names.
    * 
    * @param subNames the sub-names to combine into a single name.
    * @return the name.
    * @see #appendSuffix(String, String)
    */
   public static String assembleName(String... subNames)
   {
      if (subNames == null || subNames.length == 0)
         return null;

      String name = subNames[0];

      for (int i = 1; i < subNames.length; i++)
      {
         name = appendSuffix(name, subNames[i]);
      }

      return name;
   }

   /**
    * Appends a suffix to a name and manages the casing of the suffix as follows:
    * <ul>
    * <li>the suffix first character is modified to be lower case if the name is {@code null}, empty,
    * or if it ends with _
    * <li>the suffix first character is modified to be upper case otherwise.
    * </ul>
    * 
    * @param name   the {@code String} to append a suffix to.
    * @param suffix the suffix to append.
    * @return {@code name + suffix} with case managed for the suffix.
    */
   public static String appendSuffix(String name, String suffix)
   {
      if (name == null || name.isEmpty())
         return StringUtils.uncapitalize(suffix);
      if (suffix == null || suffix.isEmpty())
         return name;

      if (name.endsWith("_"))
         return name + StringUtils.uncapitalize(suffix);
      else
         return name + StringUtils.capitalize(suffix);
   }
}
