package us.ihmc.yoVariables.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import java.util.regex.Pattern;
import java.util.stream.Stream;

import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoSearchTools
{
   public static YoParameter findFirstParameter(String parentNameSpace, String name, Predicate<YoParameter> predicate, YoRegistry registry)
   {
      Predicate<YoVariable> yoVariablePredicate = YoVariable::isParameter;
      if (predicate != null)
         yoVariablePredicate = yoVariablePredicate.and(variable -> predicate.test(variable.getParameter()));

      YoVariable result = YoSearchTools.findFirstVariable(parentNameSpace, name, yoVariablePredicate, registry);
      if (result == null)
         return null;
      else
         return result.getParameter();
   }

   public static YoVariable findFirstVariable(String parentNameSpace, String name, Predicate<YoVariable> predicate, YoRegistry registry)
   {
      if (parentNameSpace == null || registry.getNameSpace().endsWith(parentNameSpace))
      {
         YoVariable variable = registry.getVariable(name);
         if (variable != null)
         {
            if (predicate == null || predicate.test(variable))
               return variable;
         }
      }

      for (YoRegistry child : registry.getChildren())
      {
         YoVariable variable = findFirstVariable(parentNameSpace, name, predicate, child);
         if (variable != null)
            return variable;
      }

      return null;
   }

   public static List<YoVariable> findVariables(String parentNameSpace, String name, Predicate<YoVariable> predicate, YoRegistry registry,
                                                List<YoVariable> matchedVariablesToPack)
   {
      if (matchedVariablesToPack == null)
         matchedVariablesToPack = new ArrayList<>();
      if (parentNameSpace == null || registry.getNameSpace().endsWith(parentNameSpace))
      {
         YoVariable variable = registry.getVariable(name);
         if (variable != null)
         {
            if (predicate == null || predicate.test(variable))
               matchedVariablesToPack.add(variable);
         }
      }

      for (YoRegistry child : registry.getChildren())
      {
         findVariables(parentNameSpace, name, predicate, child, matchedVariablesToPack);
      }
      return matchedVariablesToPack;
   }

   public static YoVariable findFirstVariable(Predicate<YoVariable> filter, YoVariableHolder yoVariableHolder)
   {
      for (YoVariable variable : yoVariableHolder.getVariables())
      {
         if (filter.test(variable))
            return variable;
      }

      for (YoVariableHolder child : yoVariableHolder.getChildren())
      {
         YoVariable result = findFirstVariable(filter, child);
         if (result != null)
            return result;
      }

      return null;
   }

   public static List<YoVariable> filterVariables(Predicate<YoVariable> filter, YoVariableHolder yoVariableHolder)
   {
      return YoSearchTools.filterVariables(filter, yoVariableHolder, null);
   }

   public static List<YoVariable> filterVariables(Predicate<YoVariable> filter, YoVariableHolder yoVariableHolder, List<YoVariable> variablesToPack)
   {
      if (variablesToPack == null)
         variablesToPack = new ArrayList<>();

      for (YoVariable variable : yoVariableHolder.getVariables())
      {
         if (filter.test(variable))
            variablesToPack.add(variable);
      }

      for (YoVariableHolder child : yoVariableHolder.getChildren())
      {
         filterVariables(filter, child, variablesToPack);
      }

      return variablesToPack;
   }

   public static Predicate<YoVariable> regularExpressionFilter(String... regularExpressions)
   {
      Pattern[] patterns = Stream.of(regularExpressions).map(Pattern::compile).toArray(Pattern[]::new);
      return variable ->
      {
         for (Pattern pattern : patterns)
         {
            if (pattern.matcher(variable.getName()).matches())
               return true;
         }
         return false;
      };
   }

}
