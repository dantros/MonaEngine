#pragma once
#ifndef ROOT_H
#define ROOT_H

namespace Mona {
	class Root {
	public:
		static Root& Get()
		{
			static Root s_Instance;
			return s_Instance;
		}

		void Init() noexcept 
		{

		}
	private:
		Root() noexcept {}
		~Root() {}
	
	};
}

#endif