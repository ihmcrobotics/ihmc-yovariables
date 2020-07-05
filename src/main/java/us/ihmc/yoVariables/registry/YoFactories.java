package us.ihmc.yoVariables.registry;

public class YoFactories
{

   public static YoRegistry getOrCreateAndAddRegistry(YoRegistry startRegistry, NameSpace fullNameSpace)
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
         for (YoRegistry child : startRegistry.getChildren())
         {
            YoRegistry registry = getOrCreateAndAddRegistry(child, fullNameSpace);
            if (registry != null)
               return registry;
         }

         // If, after going through all the children, none of them match, then
         // create it here and return it.
         NameSpace nameSpaceToContinueWith = fullNameSpace.removeStart(nameSpace);
         YoRegistry registry = createChainOfRegistries(nameSpaceToContinueWith);
         startRegistry.addChild(registry);

         return getToBottomRegistry(registry);
      }

      return null;
   }

   private static YoRegistry createChainOfRegistries(NameSpace fullNameSpace)
   {
      YoRegistry rootRegistry = new YoRegistry(fullNameSpace.getRootName());
      YoRegistry current = rootRegistry;

      for (int i = 1; i < fullNameSpace.size(); i++)
      {
         YoRegistry child = new YoRegistry(fullNameSpace.getSubName(i));
         current.addChild(child);
         current = child;
      }

      return rootRegistry;
   }

   private static YoRegistry getToBottomRegistry(YoRegistry root)
   {
      if (root.getChildren().size() == 0)
         return root;
      if (root.getChildren().size() > 1)
         throw new RuntimeException("This should only be called with a new chain!!");

      return getToBottomRegistry(root.getChildren().get(0));
   }

}
