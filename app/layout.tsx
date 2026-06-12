import type {Metadata} from "next";
import AuthModal from "@/components/AuthModal";
import {AuthProvider} from "@/context/AuthContext";
import "./globals.css";
export const metadata:Metadata={title:"PromptBox - AI 提示词工具箱",description:"精选 AI 提示词，一键复制，变量填充，帮你更好地使用 ChatGPT 和 Claude"};
export default function RootLayout({children}:{children:React.ReactNode}){return <html lang="zh-CN"><body><AuthProvider>{children}<AuthModal/></AuthProvider></body></html>;}
