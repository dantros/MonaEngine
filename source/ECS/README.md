# Documentación del ECS

El presente archivo busca dar a conocer cómo funciona la estructura implementada en ECS para manejar componentes, eventos y sistemas dentro de Mona, mediante la librería EnTT. En primer lugar, los archivos `Components.hpp`, `Events.hpp` y `Systems.hpp` son archivos que importan los headers de los eventos, componentes y sistemas realizados. Debido a esto, cualquier evento, componente o sistema nuevo debe ser agregado a su respectivo archivo.

Ya hablando de la implementación en sí, se tiene el archivo `ECS.hpp` con el `ECSHandler`, que funciona como una especie de wrapper para todas las funcionalidades dentro del `ComponentManager`, `EventManager` y `SystemManager`. Cada una de estas clases busca manejar los sistemas, componentes y eventos mediante las funcionalidades que provee EnTT. Esto permite que agregar eventos, componentes y sistemas sea mucho más fácil, y el futuro usuario no tenga que modificar el contenido de estos objetos. Además, dentro de las carpetas de Components, Events y Systems están los componentes, eventos y sistemas implementados en los ejemplos `ECSExample.cpp` y `ECSBreakout.cpp`.

El módulo también permite extenderse de forma flexible. Una componente puede ser cualquier cosa con un constructor válido, un evento puede ser la instancia de cualquier clase o estructura, y se define una interfaz para los sistemas, de la cual se pueden extender nuevos sistemas según las necesidades específicas del proyecto.


Finalmente, se implementó la estructura de ECS como algo que el usuario puede o no ocupar. Es decir, queda a criterio del programador y la implementación que esté buscando crear. Para esto, se tuvo que agregar un método de `EnableECS` dentro del Mona `World`. Luego, en los GameObjects se puede obtener el objeto ECS mediante el `World`.

_💡 Algo importante de destacar es que todas las funcionalidades de ECS quedaron bajo el namespace de `MonaECS`, por lo que, para acceder a todas las funcionalidades implementadas, será necesario utilizar este namespace. Mediante este también se diferencia el EventManager, ComponentManager y SystemManager de Mona con los implementados en esta versión._
