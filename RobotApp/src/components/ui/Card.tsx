import { HTMLAttributes } from "react";

export function Card({ className = "", children, ...rest }: HTMLAttributes<HTMLDivElement>) {
  return (
    <div className={`p-7 ${className}`} {...rest}>
      {children}
    </div>
  );
}
