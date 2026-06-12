import Link from "next/link";
import {redirect} from "next/navigation";
import AdminActivateForm from "@/components/AdminActivateForm";
import {isAdminEmail} from "@/lib/admin";
import {createClient} from "@/lib/supabase/server";
export const dynamic="force-dynamic";
export default async function Activate(){const supabase=createClient();const {data:{user}}=await supabase.auth.getUser();if(!user)redirect("/");if(!isAdminEmail(user.email))return <main className="flex min-h-screen items-center justify-center bg-zinc-50 px-4"><div className="rounded-2xl bg-white p-8 text-center shadow"><h1 className="text-2xl font-bold">没有访问权限</h1><Link href="/" className="mt-6 inline-flex rounded-full bg-[#6366f1] px-6 py-3 text-white">返回首页</Link></div></main>;return <main className="min-h-screen bg-zinc-50 px-4 py-8"><div className="mx-auto max-w-2xl"><Link href="/" className="text-sm text-[#6366f1]">← 返回首页</Link><h1 className="mt-6 text-3xl font-bold">开通专业版会员</h1><p className="mt-2 text-sm text-zinc-600">输入用户注册邮箱，一键开通或顺延 30 天专业版。</p><div className="mt-6"><AdminActivateForm/></div></div></main>}
