package us.ihmc.yoVariables.registry;

public class YoFactories
{

   public static YoVariableRegistry getOrCreateAndAddRegistry(YoVariableRegistry startRegistry, NameSpace fullNameSpace)
   {
      NameSpace nameSpace = startRegistry.getNameSpace();

      if (nameSpace == null && fullNameSpace == null)
      {
         return startRegistry;
      }

      if (nameSpace != null && nameSpace.equals(fullNameSpace))
      {
         return startRegistry;
      }

      if (nameSpace == null || fullNameSpace.startsWith(nameSpace.getName()))
      {
         for (YoVariableRegistry child : startRegistry.getChildren())
         {
            YoVariableRegistry registry = getOrCreateAndAddRegistry(child, fullNameSpace);
            if (registry != null)
               return registry;
         }

         // If, after going through all the children, none of them match, then
         // create it here and return it.
         NameSpace nameSpaceToContinueWith = fullNameSpace.removeStart(nameSpace);
         YoVariableRegistry registry = createChainOfRegistries(nameSpaceToContinueWith);
         startRegistry.addChild(registry);

         return getToBottomRegistry(registry);
      }

      return null;
   }

   private static YoVariableRegistry createChainOfRegistries(NameSpace fullNameSpace)
   {
      YoVariableRegistry rootRegistry = new YoVariableRegistry(fullNameSpace.getRootName());
      YoVariableRegistry current = rootRegistry;

      for (int i = 1; i < fullNameSpace.size(); i++)
      {
         YoVariableRegistry child = new YoVariableRegistry(fullNameSpace.getSubName(i));
         current.addChild(child);
         current = child;
      }

      return rootRegistry;
   }

   private static YoVariableRegistry getToBottomRegistry(YoVariableRegistry root)
   {
      if (root.getChildren().size() == 0)
         return root;
      if (root.getChildren().size() > 1)
         throw new RuntimeException("This should only be called with a new chain!!");

      return getToBottomRegistry(root.getChildren().get(0));
   }

}
