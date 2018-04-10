package us.ihmc.yoVariables.util;

import org.apache.commons.lang3.StringUtils;

public class YoFrameVariableNameTools
{
   public static String getCommonPrefix(String... strings)
   {
      return StringUtils.getCommonPrefix(strings);
   }

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

   public static String createXName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "x", nameSuffix);
   }

   public static String createYName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "y", nameSuffix);
   }

   public static String createZName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "z", nameSuffix);
   }

   public static String createQxName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "qx", nameSuffix);
   }

   public static String createQyName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "qy", nameSuffix);
   }

   public static String createQzName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "qz", nameSuffix);
   }

   public static String createQsName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "qs", nameSuffix);
   }

   public static String createName(String namePrefix, String between, String nameSuffix)
   {
      if ((namePrefix == null) || (namePrefix.equals("") || namePrefix.endsWith("_")))
      {
         return namePrefix + StringUtils.uncapitalize(between) + nameSuffix;
      }

      else
      {
         return namePrefix + StringUtils.capitalize(between) + nameSuffix;
      }
   }
}
