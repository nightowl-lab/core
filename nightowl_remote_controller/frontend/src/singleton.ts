export function createInstance<Type>(arg: { new(): Type; }): () => Type {
    const instance = new arg();
    return () => instance;
}